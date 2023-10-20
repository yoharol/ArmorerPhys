#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      armgl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example6: imgui input");
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

  armgl::Matx2f v_p;
  v_p.resize(1, 2);
  v_p << 0.0f, 0.0f;  //
  armgl::Matx3f v_color;
  v_color.resize(1, 3);
  v_color << 1.0f, 0.2f, 0.0f;

  armgl::set_points_data(points, v_p, v_color);
  points.color = armgl::RGB(255, 0, 0);
  points.point_size = 30.0f;

  armgl::add_render_func(scene, armgl::get_render_func(points));

  armgl::add_gui_mouse_input_func(gui, [&]() {
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      ImVec2 pos = ImGui::GetMousePos();
      float x = pos.x / SCR_WIDTH;
      float y = 1.0 - pos.y / SCR_HEIGHT;
      armgl::camera2d_screen_to_world(scene.camera, x, y);
      v_p.row(0) = armgl::Vec2f(x, y);
    }
  });

  armgl::add_gui_key_input_func(gui, []() {
    if (ImGui::IsKeyPressed(ImGuiKey_W))
      std::cout << "W is pressed" << std::endl;
  });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    armgl::handle_gui_input(gui);

    armgl::set_background_RGB({244, 244, 244});

    armgl::set_points_data(points, v_p, v_color);

    armgl::render_scene(scene);
    armgl::render_gui(gui);

    glfwSwapBuffers(window);
  }
  armgl::destroy_gui(gui);
  glfwTerminate();
}