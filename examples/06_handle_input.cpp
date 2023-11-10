#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerSim/RenderCore.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      asim::create_window(SCR_WIDTH, SCR_HEIGHT, "Example6: handle input");
  float bottom = 0.0f;
  float top = 2.0f;
  float left = 0.0f;
  float right = 2.0f;

  asim::Scene scene =
      asim::create_scene(asim::default_light, asim::default_camera);
  asim::set_2d_camera(scene.camera, left, right, bottom, top);

  asim::Points points = asim::create_points();

  asim::Matx2f v_p;
  v_p.resize(1, 2);
  v_p << 0.0f, 0.0f;  //
  asim::Matx3f v_color;
  v_color.resize(1, 3);
  v_color << 1.0f, 0.2f, 0.0f;

  asim::set_points_data(points, v_p, v_color);
  points.color = asim::RGB(255, 0, 0);
  points.point_size = 10.0f;

  asim::add_render_func(scene, asim::get_render_func(points));

  asim::InputHandler& handler = asim::create_input_handler(window);
  asim::add_mouse_move_func(handler, [&](asim::InputHandler& input_handler) {
    if (input_handler.left_pressing) {
      float xpos, ypos;
      xpos = left + (right - left) * handler.xpos;
      ypos = bottom + (top - bottom) * handler.ypos;
      v_p.row(0) = asim::Vec2f(xpos, ypos);
    }
  });
  asim::add_mouse_input_func(handler, [](asim::InputHandler& input_handler,
                                         int button, int action) {
    std::cout << "mouse button event: " << button << " " << action << std::endl;
  });
  asim::add_key_input_func(
      handler, [](asim::InputHandler& input_handler, int key, int action) {
        std::cout << "key event: " << key << " " << action << std::endl;
      });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    asim::set_background_RGB({244, 244, 244});

    asim::set_points_data(points, v_p, v_color);

    asim::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}