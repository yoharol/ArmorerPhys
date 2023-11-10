#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include <igl/readOBJ.h>

#include "ArmorerSim/RenderCore.h"

// settings
const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 800;

int main() {
  GLFWwindow *window =
      asim::create_window(SCR_WIDTH, SCR_HEIGHT, "Example4: Pipeline");

  asim::DiffuseMaterial material{
      {255, 255, 255},  // diffuse color
      {255, 255, 255},  // specular color
      0.5f              // specular strength
  };
  asim::Scene scene = asim::create_scene(
      asim::Light{
          {242, 242, 242},     // light color
          {9, 5, 88},          // ambient color
          {0.0f, 0.35f, 5.0f}  // light position
      },
      asim::create_camera(
          {2.0f, 0.0f, -2.0f},                  // camera position
          {0.0f, 0.35f, 0.0f},                  // camera target
          {0.0f, 1.0f, 0.0f},                   // camera up axis
          float(SCR_WIDTH) / float(SCR_HEIGHT)  // camera aspect
          ));
  asim::Gui gui = asim::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 100;

  asim::Vec3f diffuse_color(0.0f, 211.f / 255.f, 239.f / 255.f);
  asim::add_gui_func(gui, [&diffuse_color]() {
    ImGui::Text("Diffuse Color:");
    ImGui::ColorEdit3("#c1", diffuse_color.data());
  });

  asim::MatxXf V;
  asim::MatxXi F;
  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);
  int n_vertices = V.rows();
  int n_faces = F.rows();

  asim::MatxXf v_p(4, 3);
  // select 289, 577, 572, 284 vertex from V to v_p
  v_p.row(0) = V.row(289);
  v_p.row(1) = V.row(577);
  v_p.row(2) = V.row(572);
  v_p.row(3) = V.row(284);

  asim::Points points = asim::create_points();
  points.color = {255, 0, 0};
  points.point_size = 10.0f;
  asim::Lines lines = asim::create_lines();
  lines.color = {0, 155, 155};
  lines.mode = GL_LINE_LOOP;

  asim::set_points_data(points, v_p, asim::MatxXf());
  asim::set_lines_data(lines, v_p, asim::MatxXf());

  asim::DiffuseMesh mesh = asim::create_diffuse_mesh(material);
  asim::set_mesh_data(mesh, V, F);

  asim::add_render_func(scene, asim::get_render_func(mesh));
  asim::add_render_func(scene, asim::get_render_func(lines),
                         false);  // disable depth test
  asim::add_render_func(scene, asim::get_render_func(points),
                         false);  // disable depth test

  asim::set_wireframe_mode(false);

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    asim::set_background_RGB(asim::RGB(250, 240, 228));

    float curr_time = glfwGetTime();
    asim::orbit_camera_control(window, scene.camera, 10.0,
                                curr_time - prev_time);
    prev_time = curr_time;
    scene.light.position =
        asim::Vec3f(5.0f * sin((curr_time - start_time) * 1.0f), 0.35f,
                     5.0f * cos((curr_time - start_time) * 1.0f));

    asim::render_scene(scene);
    asim::render_gui(gui);

    asim::use_program(mesh.program);
    asim::set_uniform_float3(mesh.program, "diffuseColor", diffuse_color);
    asim::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  asim::delete_mesh(mesh);
  asim::destroy_gui(gui);
  glfwTerminate();
  return 0;
}
