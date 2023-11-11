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
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT, "Example6: imgui input");
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

  aphys::Matx2f v_p;
  v_p.resize(1, 2);
  v_p << 0.0f, 0.0f;  //
  aphys::Matx3f v_color;
  v_color.resize(1, 3);
  v_color << 1.0f, 0.2f, 0.0f;

  aphys::set_points_data(points, v_p, v_color);
  points.color = aphys::RGB(255, 0, 0);
  points.point_size = 30.0f;

  aphys::add_render_func(scene, aphys::get_render_func(points));

  aphys::add_gui_mouse_input_func(gui, [&]() {
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      ImVec2 pos = ImGui::GetMousePos();
      float x = pos.x / SCR_WIDTH;
      float y = 1.0 - pos.y / SCR_HEIGHT;
      aphys::camera2d_screen_to_world(scene.camera, x, y);
      v_p.row(0) = aphys::Vec2f(x, y);
    }
  });

  aphys::add_gui_key_input_func(gui, []() {
    if (ImGui::IsKeyPressed(ImGuiKey_W))
      std::cout << "W is pressed" << std::endl;
  });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    aphys::handle_gui_input(gui);

    aphys::set_background_RGB({244, 244, 244});

    aphys::set_points_data(points, v_p, v_color);

    aphys::render_scene(scene);
    aphys::render_gui(gui);

    glfwSwapBuffers(window);
  }
  aphys::destroy_gui(gui);
  glfwTerminate();
}