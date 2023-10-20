#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      armgl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example6: handle input");
  float bottom = 0.0f;
  float top = 2.0f;
  float left = 0.0f;
  float right = 2.0f;

  armgl::Scene scene =
      armgl::create_scene(armgl::default_light, armgl::default_camera);
  armgl::set_2d_camera(scene.camera, left, right, bottom, top);

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

  armgl::InputHandler& handler = armgl::create_input_handler(window);
  armgl::add_mouse_move_func(handler, [&](armgl::InputHandler& input_handler) {
    if (input_handler.left_pressing) {
      float xpos, ypos;
      xpos = left + (right - left) * handler.xpos;
      ypos = bottom + (top - bottom) * handler.ypos;
      v_p.row(0) = armgl::Vec2f(xpos, ypos);
    }
  });
  armgl::add_mouse_input_func(handler, [](armgl::InputHandler& input_handler,
                                          int button, int action) {
    std::cout << "mouse button event: " << button << " " << action << std::endl;
  });
  armgl::add_key_input_func(
      handler, [](armgl::InputHandler& input_handler, int key, int action) {
        std::cout << "key event: " << key << " " << action << std::endl;
      });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    armgl::set_background_RGB({244, 244, 244});

    armgl::set_points_data(points, v_p, v_color);

    armgl::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}