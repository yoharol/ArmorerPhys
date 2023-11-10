#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerSim/RenderCore.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      asim::create_window(SCR_WIDTH, SCR_HEIGHT, "Example6: imgui input");
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

  asim::Matx2f v_p;
  v_p.resize(1, 2);
  v_p << 0.0f, 0.0f;  //
  asim::Matx3f v_color;
  v_color.resize(1, 3);
  v_color << 1.0f, 0.2f, 0.0f;

  asim::set_points_data(points, v_p, v_color);
  points.color = asim::RGB(255, 0, 0);
  points.point_size = 30.0f;

  asim::add_render_func(scene, asim::get_render_func(points));

  asim::add_gui_mouse_input_func(gui, [&]() {
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      ImVec2 pos = ImGui::GetMousePos();
      float x = pos.x / SCR_WIDTH;
      float y = 1.0 - pos.y / SCR_HEIGHT;
      asim::camera2d_screen_to_world(scene.camera, x, y);
      v_p.row(0) = asim::Vec2f(x, y);
    }
  });

  asim::add_gui_key_input_func(gui, []() {
    if (ImGui::IsKeyPressed(ImGuiKey_W))
      std::cout << "W is pressed" << std::endl;
  });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    asim::handle_gui_input(gui);

    asim::set_background_RGB({244, 244, 244});

    asim::set_points_data(points, v_p, v_color);

    asim::render_scene(scene);
    asim::render_gui(gui);

    glfwSwapBuffers(window);
  }
  asim::destroy_gui(gui);
  glfwTerminate();
}