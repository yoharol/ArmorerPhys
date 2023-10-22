#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      armgl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example5: 2d game");
  float bottom = 0.0f;
  float top = 2.0f;
  float left = 0.0f;
  float right = 2.0f;

  armgl::Scene scene =
      armgl::create_scene(armgl::default_light, armgl::default_camera);
  armgl::set_2d_camera(scene.camera, left, right, bottom, top);
  armgl::Gui gui = armgl::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 300;

  armgl::Points points = armgl::create_points();
  armgl::Lines lines = armgl::create_lines();

  armgl::Matx2f v_p;
  v_p.resize(2, 2);
  v_p << 1.0f, 1.0f,  //
      1.0f, 0.5f;     //
  armgl::Matx3f v_color;
  v_color.resize(2, 3);
  v_color << 0.0f, 0.0f, 0.0f,  //
      1.0f, 0.2f, 0.0f;         //
  armgl::Matx2f v_p_ref = v_p;
  float l0 = (v_p.row(0) - v_p.row(1)).norm();
  armgl::Matx2f v_vel;
  v_vel.resize(2, 2);
  v_vel << 0.0f, 0.0f,  //
      0.0f, 0.0f;       //
  armgl::set_points_data(points, v_p, v_color);
  points.color = armgl::RGB(255, 0, 0);
  points.point_size = 10.0f;
  armgl::set_lines_data(lines, v_p, v_color);
  lines.color = armgl::RGB(255, 0, 0);
  lines.width = 1.0f;

  armgl::Vec2f gravity(0.0f, -10.0f);
  float mass = 1.0f;
  float stiffness = 50.0f;
  float damping = 0.5f;

  auto reset = [&]() {
    v_p = v_p_ref;
    v_vel.setZero();
    armgl::set_points_data(points, v_p, v_color);
    armgl::set_lines_data(lines, v_p, v_color);
  };

  armgl::add_gui_func(gui, [&gravity, &stiffness, &mass, &damping, &reset]() {
    ImGui::Text("Parameters:");
    ImGui::InputFloat2("g", gravity.data());
    ImGui::InputFloat("k", &stiffness);
    ImGui::InputFloat("m", &mass);
    ImGui::InputFloat("beta", &damping);
    if (ImGui::Button("Reset")) {
      reset();
    }
  });
  armgl::add_render_func(scene, armgl::get_render_func(points));
  armgl::add_render_func(scene, armgl::get_render_func(lines));

  glfwSwapInterval(1);

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    armgl::set_background_RGB({244, 244, 244});

    float delta_time = glfwGetTime() - prev_time;
    prev_time = glfwGetTime();
    float dt = delta_time / 10.0f;

    for (int _ = 0; _ < 10; _++) {
      armgl::Vec2f dx = v_p.row(0) - v_p.row(1);
      armgl::Vec2f force = gravity * mass + stiffness * (dx.norm() - l0) * dx;
      armgl::Vec2f acc = force / mass;
      v_vel.row(1).transpose() += acc * dt;
      v_vel *= exp(-damping * dt);
      v_p += v_vel * dt;
    }

    armgl::set_points_data(points, v_p, v_color);
    armgl::set_lines_data(lines, v_p, v_color);

    // armgl::Mat2fToList3f(v_p, pos);
    armgl::render_scene(scene);
    armgl::render_gui(gui);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  armgl::destroy_gui(gui);
  glfwTerminate();
}