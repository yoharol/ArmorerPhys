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
      glrender::create_window(SCR_WIDTH, SCR_HEIGHT, "Example3: Lighting");
  glrender::init_glad();

  glrender::DiffuseMaterial material{{255, 255, 255},  //
                                     {255, 255, 255},  //
                                     0.5f};
  glrender::Scene scene = glrender::create_scene(
      glrender::Light{{242, 76, 61}, {9, 5, 128}, {0.0f, 0.35f, 5.0f}},
      glrender::create_camera({2.0f, 0.0f, -2.0f}, {0.0f, 0.35f, 0.0f},
                              {0.0f, 1.0f, 0.0f},
                              float(SCR_WIDTH) / float(SCR_HEIGHT)));

  glrender::MatXf V;
  glrender::MatXi F;
  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);
  int n_vertices = V.rows();
  int n_faces = F.rows();

  glrender::DiffuseMesh mesh = glrender::create_diffuse_mesh(material);
  glrender::set_mesh_data(mesh, V, F);
  scene.render_funcs.push_back(glrender::get_render_func(mesh));

  // glrender::set_wireframe_mode(true);
  glrender::set_wireframe_mode(false);

  // glrender::update_camera(camera);

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

    for (auto render_func : scene.render_funcs) {
      render_func(scene);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glrender::delete_mesh(mesh);

  glfwTerminate();
  return 0;
}
