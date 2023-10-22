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
      armgl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example4: Pipeline");

  armgl::DiffuseMaterial material{
      {255, 255, 255},  // diffuse color
      {255, 255, 255},  // specular color
      0.5f              // specular strength
  };
  armgl::Scene scene = armgl::create_scene(
      armgl::Light{
          {242, 242, 242},     // light color
          {9, 5, 88},          // ambient color
          {0.0f, 0.35f, 5.0f}  // light position
      },
      armgl::create_camera(
          {2.0f, 0.0f, -2.0f},                  // camera position
          {0.0f, 0.35f, 0.0f},                  // camera target
          {0.0f, 1.0f, 0.0f},                   // camera up axis
          float(SCR_WIDTH) / float(SCR_HEIGHT)  // camera aspect
          ));
  armgl::Gui gui = armgl::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 100;

  armgl::Vec3f diffuse_color(0.0f, 211.f / 255.f, 239.f / 255.f);
  armgl::add_gui_func(gui, [&diffuse_color]() {
    ImGui::Text("Diffuse Color:");
    ImGui::ColorEdit3("#c1", diffuse_color.data());
  });

  armgl::MatxXf V;
  armgl::MatxXi F;
  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);
  int n_vertices = V.rows();
  int n_faces = F.rows();

  armgl::MatxXf v_p(4, 3);
  // select 289, 577, 572, 284 vertex from V to v_p
  v_p.row(0) = V.row(289);
  v_p.row(1) = V.row(577);
  v_p.row(2) = V.row(572);
  v_p.row(3) = V.row(284);

  armgl::Points points = armgl::create_points();
  points.color = {255, 0, 0};
  points.point_size = 10.0f;
  armgl::Lines lines = armgl::create_lines();
  lines.color = {0, 155, 155};
  lines.mode = GL_LINE_LOOP;

  armgl::set_points_data(points, v_p, armgl::MatxXf());
  armgl::set_lines_data(lines, v_p, armgl::MatxXf());

  armgl::DiffuseMesh mesh = armgl::create_diffuse_mesh(material);
  armgl::set_mesh_data(mesh, V, F);

  armgl::add_render_func(scene, armgl::get_render_func(mesh));
  armgl::add_render_func(scene, armgl::get_render_func(lines),
                         false);  // disable depth test
  armgl::add_render_func(scene, armgl::get_render_func(points),
                         false);  // disable depth test

  armgl::set_wireframe_mode(false);

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    armgl::set_background_RGB(armgl::RGB(250, 240, 228));

    float curr_time = glfwGetTime();
    armgl::orbit_camera_control(window, scene.camera, 10.0,
                                curr_time - prev_time);
    prev_time = curr_time;
    scene.light.position =
        armgl::Vec3f(5.0f * sin((curr_time - start_time) * 1.0f), 0.35f,
                     5.0f * cos((curr_time - start_time) * 1.0f));

    armgl::render_scene(scene);
    armgl::render_gui(gui);

    armgl::use_program(mesh.program);
    armgl::set_uniform_float3(mesh.program, "diffuseColor", diffuse_color);
    armgl::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  armgl::delete_mesh(mesh);
  armgl::destroy_gui(gui);
  glfwTerminate();
  return 0;
}
