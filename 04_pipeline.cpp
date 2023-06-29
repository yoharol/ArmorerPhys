#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include <igl/readOBJ.h>

#include "gl_render.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  glrender::init_glfw();
  GLFWwindow *window =
      glrender::create_window(SCR_WIDTH, SCR_HEIGHT, "Example4: Pipeline");
  glrender::init_glad();

  glrender::DiffuseMaterial material{
      {255, 255, 255},  // diffuse color
      {255, 255, 255},  // specular color
      0.5f              // specular strength
  };
  glrender::Scene scene = glrender::create_scene(
      glrender::Light{
          {242, 76, 61},       // light color
          {9, 5, 128},         // ambient color
          {0.0f, 0.35f, 5.0f}  // light position
      },
      glrender::create_camera(
          {2.0f, 0.0f, -2.0f},                  // camera position
          {0.0f, 0.35f, 0.0f},                  // camera target
          {0.0f, 1.0f, 0.0f},                   // camera up axis
          float(SCR_WIDTH) / float(SCR_HEIGHT)  // camera aspect
          ));

  glrender::MatXf V;
  glrender::MatXi F;
  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);
  int n_vertices = V.rows();
  int n_faces = F.rows();

  std::vector<glrender::Vec3f> points;
  points.resize(4);
  points[0] = V.row(289).transpose();
  points[1] = V.row(577).transpose();
  points[2] = V.row(572).transpose();
  points[3] = V.row(284).transpose();
  glrender::ContinuousLines lines{points, {}, {255, 0, 0}, 100.0f, true};

  glrender::DiffuseMesh mesh = glrender::create_diffuse_mesh(material);
  glrender::set_mesh_data(mesh, V, F);

  glrender::add_render_func(scene, glrender::get_render_func(mesh));
  glrender::add_render_func(scene, glrender::get_render_func(lines));

  glrender::set_wireframe_mode(false);

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    glrender::set_background_RGB(glrender::RGB(250, 240, 228));

    float curr_time = glfwGetTime();
    glrender::orbit_camera_control(window, scene.camera, 10.0,
                                   curr_time - prev_time);
    prev_time = curr_time;
    scene.light.position =
        glrender::Vec3f(5.0f * sin((curr_time - start_time) * 1.0f), 0.35f,
                        5.0f * cos((curr_time - start_time) * 1.0f));

    glrender::render_scene(scene);

    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3ub(255, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glEnd();

    glLineWidth(10.0f);
    glColor3ub(255, 0, 0);
    glBegin(GL_LINES);
    // glVertex3f(-0.09f, -0.73f, 0.62f);
    // glVertex3f(-0.3f, -0.64f, 0.68f);
    glVertex2f(0.0f, 0.0f);
    glVertex2f(1.0f, 1.0f);
    // glrender::draw_line({-0.09f, -0.73f, 0.62f}, {-0.3f, -0.64f, 0.68f},
    //                     glrender::RGB(255, 0, 0), 10.0f);
    glEnd();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glrender::delete_mesh(mesh);

  glfwTerminate();
  return 0;
}
