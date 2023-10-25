#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      agl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example6: handle input");
  float bottom = 0.0f;
  float top = 2.0f;
  float left = 0.0f;
  float right = 2.0f;

  agl::Scene scene =
      agl::create_scene(agl::default_light, agl::default_camera);
  agl::set_2d_camera(scene.camera, left, right, bottom, top);

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

  agl::InputHandler& handler = agl::create_input_handler(window);
  agl::add_mouse_move_func(handler, [&](agl::InputHandler& input_handler) {
    if (input_handler.left_pressing) {
      float xpos, ypos;
      xpos = left + (right - left) * handler.xpos;
      ypos = bottom + (top - bottom) * handler.ypos;
      v_p.row(0) = agl::Vec2f(xpos, ypos);
    }
  });
  agl::add_mouse_input_func(handler, [](agl::InputHandler& input_handler,
                                          int button, int action) {
    std::cout << "mouse button event: " << button << " " << action << std::endl;
  });
  agl::add_key_input_func(
      handler, [](agl::InputHandler& input_handler, int key, int action) {
        std::cout << "key event: " << key << " " << action << std::endl;
      });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    agl::set_background_RGB({244, 244, 244});

    agl::set_points_data(points, v_p, v_color);

    agl::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}