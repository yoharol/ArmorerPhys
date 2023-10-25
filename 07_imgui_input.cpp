#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      agl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example6: imgui input");
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

  agl::Matx2f v_p;
  v_p.resize(1, 2);
  v_p << 0.0f, 0.0f;  //
  agl::Matx3f v_color;
  v_color.resize(1, 3);
  v_color << 1.0f, 0.2f, 0.0f;

  agl::set_points_data(points, v_p, v_color);
  points.color = agl::RGB(255, 0, 0);
  points.point_size = 30.0f;

  agl::add_render_func(scene, agl::get_render_func(points));

  agl::add_gui_mouse_input_func(gui, [&]() {
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      ImVec2 pos = ImGui::GetMousePos();
      float x = pos.x / SCR_WIDTH;
      float y = 1.0 - pos.y / SCR_HEIGHT;
      agl::camera2d_screen_to_world(scene.camera, x, y);
      v_p.row(0) = agl::Vec2f(x, y);
    }
  });

  agl::add_gui_key_input_func(gui, []() {
    if (ImGui::IsKeyPressed(ImGuiKey_W))
      std::cout << "W is pressed" << std::endl;
  });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    agl::handle_gui_input(gui);

    agl::set_background_RGB({244, 244, 244});

    agl::set_points_data(points, v_p, v_color);

    agl::render_scene(scene);
    agl::render_gui(gui);

    glfwSwapBuffers(window);
  }
  agl::destroy_gui(gui);
  glfwTerminate();
}