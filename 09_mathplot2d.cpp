#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 400;

int main() {
  GLFWwindow* window = armgl::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example7: Affine2d Visualization");

  armgl::Scene scene =
      armgl::create_scene(armgl::default_light, armgl::default_camera);
  armgl::set_2d_camera(scene.camera, -2.0f, 2.0f, -1.0f, 1.0f);
  armgl::Gui gui = armgl::create_gui(window, "gui");
  gui.width = 0;
  gui.height = 0;

  armgl::Matx2f v_p(2, 2);
  v_p << 0.0f, 0.0f,  //
      1.0f, 1.0f;     //

  armgl::Points points = armgl::create_points();
  armgl::Lines ruler =
      armgl::create_ruler2d(-2.0f, -2.0f, 2.0f, 2.0f, armgl::RGB(200, 34, 0));
  armgl::Lines axis =
      armgl::create_axis2d(-2.0f, 2.0f, -1.0f, 1.0f, armgl::RGB(0, 34, 167));

  armgl::add_render_func(scene, armgl::get_render_func(points));
  armgl::add_render_func(scene, armgl::get_render_func(ruler));
  armgl::add_render_func(scene, armgl::get_render_func(axis));

  armgl::InputHandler handler = armgl::create_input_handler(window);

  armgl::add_gui_mouse_input_func(gui, [&]() {
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      ImVec2 pos = ImGui::GetMousePos();
      float x = pos.x / SCR_WIDTH;
      float y = 1.0 - pos.y / SCR_HEIGHT;
      armgl::camera2d_screen_to_world(scene.camera, x, y);
      armgl::Vec2f p(x, y);
      if ((v_p.row(0) - p.transpose()).norm() <
          (v_p.row(1) - p.transpose()).norm()) {
        v_p.row(0) = p;
      } else {
        v_p.row(1) = p;
      }
    }
  });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    armgl::handle_gui_input(gui);

    armgl::set_points_data(points, v_p, armgl::MatxXf());

    armgl::set_background_RGB({244, 244, 244});
    armgl::render_scene(scene);
    armgl::render_gui(gui);

    glfwSwapBuffers(window);
  }
  armgl::destroy_gui(gui);
  glfwTerminate();
}