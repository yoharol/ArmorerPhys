#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      agl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example5: 2d game");
  float bottom = 0.0f;
  float top = 2.0f;
  float left = 0.0f;
  float right = 2.0f;

  agl::Scene scene =
      agl::create_scene(agl::default_light, agl::default_camera);
  agl::set_2d_camera(scene.camera, left, right, bottom, top);
  agl::Gui gui = agl::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 300;

  agl::Points points = agl::create_points();
  agl::Lines lines = agl::create_lines();

  agl::Matx2f v_p;
  v_p.resize(2, 2);
  v_p << 1.0f, 1.0f,  //
      1.0f, 0.5f;     //
  agl::Matx3f v_color;
  v_color.resize(2, 3);
  v_color << 0.0f, 0.0f, 0.0f,  //
      1.0f, 0.2f, 0.0f;         //
  agl::Matx2f v_p_ref = v_p;
  float l0 = (v_p.row(0) - v_p.row(1)).norm();
  agl::Matx2f v_vel;
  v_vel.resize(2, 2);
  v_vel << 0.0f, 0.0f,  //
      0.0f, 0.0f;       //
  agl::set_points_data(points, v_p, v_color);
  points.color = agl::RGB(255, 0, 0);
  points.point_size = 10.0f;
  agl::set_lines_data(lines, v_p, v_color);
  lines.color = agl::RGB(255, 0, 0);
  lines.width = 1.0f;

  agl::Vec2f gravity(0.0f, -10.0f);
  float mass = 1.0f;
  float stiffness = 50.0f;
  float damping = 0.5f;

  auto reset = [&]() {
    v_p = v_p_ref;
    v_vel.setZero();
    agl::set_points_data(points, v_p, v_color);
    agl::set_lines_data(lines, v_p, v_color);
  };

  agl::add_gui_func(gui, [&gravity, &stiffness, &mass, &damping, &reset]() {
    ImGui::Text("Parameters:");
    ImGui::InputFloat2("g", gravity.data());
    ImGui::InputFloat("k", &stiffness);
    ImGui::InputFloat("m", &mass);
    ImGui::InputFloat("beta", &damping);
    if (ImGui::Button("Reset")) {
      reset();
    }
  });
  agl::add_render_func(scene, agl::get_render_func(points));
  agl::add_render_func(scene, agl::get_render_func(lines));

  glfwSwapInterval(1);

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    agl::set_background_RGB({244, 244, 244});

    float delta_time = glfwGetTime() - prev_time;
    prev_time = glfwGetTime();
    float dt = delta_time / 10.0f;

    for (int _ = 0; _ < 10; _++) {
      agl::Vec2f dx = v_p.row(0) - v_p.row(1);
      agl::Vec2f force = gravity * mass + stiffness * (dx.norm() - l0) * dx;
      agl::Vec2f acc = force / mass;
      v_vel.row(1).transpose() += acc * dt;
      v_vel *= exp(-damping * dt);
      v_p += v_vel * dt;
    }

    agl::set_points_data(points, v_p, v_color);
    agl::set_lines_data(lines, v_p, v_color);

    // agl::Mat2fToList3f(v_p, pos);
    agl::render_scene(scene);
    agl::render_gui(gui);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  agl::destroy_gui(gui);
  glfwTerminate();
}