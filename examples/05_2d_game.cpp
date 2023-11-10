#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerSim/RenderCore.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      asim::create_window(SCR_WIDTH, SCR_HEIGHT, "Example5: 2d game");
  float bottom = 0.0f;
  float top = 2.0f;
  float left = 0.0f;
  float right = 2.0f;

  asim::Scene scene =
      asim::create_scene(asim::default_light, asim::default_camera);
  asim::set_2d_camera(scene.camera, left, right, bottom, top);
  asim::Gui gui = asim::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 300;

  asim::Points points = asim::create_points();
  asim::Lines lines = asim::create_lines();

  asim::Matx2f v_p;
  v_p.resize(2, 2);
  v_p << 1.0f, 1.0f,  //
      1.0f, 0.5f;     //
  asim::Matx3f v_color;
  v_color.resize(2, 3);
  v_color << 0.0f, 0.0f, 0.0f,  //
      1.0f, 0.2f, 0.0f;         //
  asim::Matx2f v_p_ref = v_p;
  float l0 = (v_p.row(0) - v_p.row(1)).norm();
  asim::Matx2f v_vel;
  v_vel.resize(2, 2);
  v_vel << 0.0f, 0.0f,  //
      0.0f, 0.0f;       //
  asim::set_points_data(points, v_p, v_color);
  points.color = asim::RGB(255, 0, 0);
  points.point_size = 10.0f;
  asim::set_lines_data(lines, v_p, v_color);
  lines.color = asim::RGB(255, 0, 0);
  lines.width = 1.0f;

  asim::Vec2f gravity(0.0f, -10.0f);
  float mass = 1.0f;
  float stiffness = 50.0f;
  float damping = 0.5f;

  auto reset = [&]() {
    v_p = v_p_ref;
    v_vel.setZero();
    asim::set_points_data(points, v_p, v_color);
    asim::set_lines_data(lines, v_p, v_color);
  };

  asim::add_gui_func(gui, [&gravity, &stiffness, &mass, &damping, &reset]() {
    ImGui::Text("Parameters:");
    ImGui::InputFloat2("g", gravity.data());
    ImGui::InputFloat("k", &stiffness);
    ImGui::InputFloat("m", &mass);
    ImGui::InputFloat("beta", &damping);
    if (ImGui::Button("Reset")) {
      reset();
    }
  });
  asim::add_render_func(scene, asim::get_render_func(points));
  asim::add_render_func(scene, asim::get_render_func(lines));

  glfwSwapInterval(1);

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    asim::set_background_RGB({244, 244, 244});

    float delta_time = glfwGetTime() - prev_time;
    prev_time = glfwGetTime();
    float dt = delta_time / 10.0f;

    for (int _ = 0; _ < 10; _++) {
      asim::Vec2f dx = v_p.row(0) - v_p.row(1);
      asim::Vec2f force = gravity * mass + stiffness * (dx.norm() - l0) * dx;
      asim::Vec2f acc = force / mass;
      v_vel.row(1).transpose() += acc * dt;
      v_vel *= exp(-damping * dt);
      v_p += v_vel * dt;
    }

    asim::set_points_data(points, v_p, v_color);
    asim::set_lines_data(lines, v_p, v_color);

    // asim::Mat2fToList3f(v_p, pos);
    asim::render_scene(scene);
    asim::render_gui(gui);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  asim::destroy_gui(gui);
  glfwTerminate();
}