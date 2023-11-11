#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT, "Example5: 2d game");
  float bottom = 0.0f;
  float top = 2.0f;
  float left = 0.0f;
  float right = 2.0f;

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, left, right, bottom, top);
  aphys::Gui gui = aphys::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 300;

  aphys::Points points = aphys::create_points();
  aphys::Lines lines = aphys::create_lines();

  aphys::Matx2f v_p;
  v_p.resize(2, 2);
  v_p << 1.0f, 1.0f,  //
      1.0f, 0.5f;     //
  aphys::Matx3f v_color;
  v_color.resize(2, 3);
  v_color << 0.0f, 0.0f, 0.0f,  //
      1.0f, 0.2f, 0.0f;         //
  aphys::Matx2f v_p_ref = v_p;
  float l0 = (v_p.row(0) - v_p.row(1)).norm();
  aphys::Matx2f v_vel;
  v_vel.resize(2, 2);
  v_vel << 0.0f, 0.0f,  //
      0.0f, 0.0f;       //
  aphys::set_points_data(points, v_p, v_color);
  points.color = aphys::RGB(255, 0, 0);
  points.point_size = 10.0f;
  aphys::set_lines_data(lines, v_p, v_color);
  lines.color = aphys::RGB(255, 0, 0);
  lines.width = 1.0f;

  aphys::Vec2f gravity(0.0f, -10.0f);
  float mass = 1.0f;
  float stiffness = 50.0f;
  float damping = 0.5f;

  auto reset = [&]() {
    v_p = v_p_ref;
    v_vel.setZero();
    aphys::set_points_data(points, v_p, v_color);
    aphys::set_lines_data(lines, v_p, v_color);
  };

  aphys::add_gui_func(gui, [&gravity, &stiffness, &mass, &damping, &reset]() {
    ImGui::Text("Parameters:");
    ImGui::InputFloat2("g", gravity.data());
    ImGui::InputFloat("k", &stiffness);
    ImGui::InputFloat("m", &mass);
    ImGui::InputFloat("beta", &damping);
    if (ImGui::Button("Reset")) {
      reset();
    }
  });
  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(lines));

  glfwSwapInterval(1);

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    aphys::set_background_RGB({244, 244, 244});

    float delta_time = glfwGetTime() - prev_time;
    prev_time = glfwGetTime();
    float dt = delta_time / 10.0f;

    for (int _ = 0; _ < 10; _++) {
      aphys::Vec2f dx = v_p.row(0) - v_p.row(1);
      aphys::Vec2f force = gravity * mass + stiffness * (dx.norm() - l0) * dx;
      aphys::Vec2f acc = force / mass;
      v_vel.row(1).transpose() += acc * dt;
      v_vel *= exp(-damping * dt);
      v_p += v_vel * dt;
    }

    aphys::set_points_data(points, v_p, v_color);
    aphys::set_lines_data(lines, v_p, v_color);

    // aphys::Mat2fToList3f(v_p, pos);
    aphys::render_scene(scene);
    aphys::render_gui(gui);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  aphys::destroy_gui(gui);
  glfwTerminate();
}