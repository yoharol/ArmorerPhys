#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include <igl/readOBJ.h>

#include "ArmorerGL.h"

// settings
const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 800;

int main() {
  GLFWwindow *window =
      agl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example4: Pipeline");

  agl::DiffuseMaterial material{
      {255, 255, 255},  // diffuse color
      {255, 255, 255},  // specular color
      0.5f              // specular strength
  };
  agl::Scene scene = agl::create_scene(
      agl::Light{
          {242, 242, 242},     // light color
          {9, 5, 88},          // ambient color
          {0.0f, 0.35f, 5.0f}  // light position
      },
      agl::create_camera(
          {2.0f, 0.0f, -2.0f},                  // camera position
          {0.0f, 0.35f, 0.0f},                  // camera target
          {0.0f, 1.0f, 0.0f},                   // camera up axis
          float(SCR_WIDTH) / float(SCR_HEIGHT)  // camera aspect
          ));
  agl::Gui gui = agl::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 100;

  agl::Vec3f diffuse_color(0.0f, 211.f / 255.f, 239.f / 255.f);
  agl::add_gui_func(gui, [&diffuse_color]() {
    ImGui::Text("Diffuse Color:");
    ImGui::ColorEdit3("#c1", diffuse_color.data());
  });

  agl::MatxXf V;
  agl::MatxXi F;
  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);
  int n_vertices = V.rows();
  int n_faces = F.rows();

  agl::MatxXf v_p(4, 3);
  // select 289, 577, 572, 284 vertex from V to v_p
  v_p.row(0) = V.row(289);
  v_p.row(1) = V.row(577);
  v_p.row(2) = V.row(572);
  v_p.row(3) = V.row(284);

  agl::Points points = agl::create_points();
  points.color = {255, 0, 0};
  points.point_size = 10.0f;
  agl::Lines lines = agl::create_lines();
  lines.color = {0, 155, 155};
  lines.mode = GL_LINE_LOOP;

  agl::set_points_data(points, v_p, agl::MatxXf());
  agl::set_lines_data(lines, v_p, agl::MatxXf());

  agl::DiffuseMesh mesh = agl::create_diffuse_mesh(material);
  agl::set_mesh_data(mesh, V, F);

  agl::add_render_func(scene, agl::get_render_func(mesh));
  agl::add_render_func(scene, agl::get_render_func(lines),
                         false);  // disable depth test
  agl::add_render_func(scene, agl::get_render_func(points),
                         false);  // disable depth test

  agl::set_wireframe_mode(false);

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    agl::set_background_RGB(agl::RGB(250, 240, 228));

    float curr_time = glfwGetTime();
    agl::orbit_camera_control(window, scene.camera, 10.0,
                                curr_time - prev_time);
    prev_time = curr_time;
    scene.light.position =
        agl::Vec3f(5.0f * sin((curr_time - start_time) * 1.0f), 0.35f,
                     5.0f * cos((curr_time - start_time) * 1.0f));

    agl::render_scene(scene);
    agl::render_gui(gui);

    agl::use_program(mesh.program);
    agl::set_uniform_float3(mesh.program, "diffuseColor", diffuse_color);
    agl::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  agl::delete_mesh(mesh);
  agl::destroy_gui(gui);
  glfwTerminate();
  return 0;
}
