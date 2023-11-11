#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerPhys/RenderCore.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT, "Example6: handle input");
  float bottom = 0.0f;
  float top = 2.0f;
  float left = 0.0f;
  float right = 2.0f;

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, left, right, bottom, top);

  aphys::Points points = aphys::create_points();

  aphys::Matx2f v_p;
  v_p.resize(1, 2);
  v_p << 0.0f, 0.0f;  //
  aphys::Matx3f v_color;
  v_color.resize(1, 3);
  v_color << 1.0f, 0.2f, 0.0f;

  aphys::set_points_data(points, v_p, v_color);
  points.color = aphys::RGB(255, 0, 0);
  points.point_size = 10.0f;

  aphys::add_render_func(scene, aphys::get_render_func(points));

  aphys::InputHandler& handler = aphys::create_input_handler(window);
  aphys::add_mouse_move_func(handler, [&](aphys::InputHandler& input_handler) {
    if (input_handler.left_pressing) {
      float xpos, ypos;
      xpos = left + (right - left) * handler.xpos;
      ypos = bottom + (top - bottom) * handler.ypos;
      v_p.row(0) = aphys::Vec2f(xpos, ypos);
    }
  });
  aphys::add_mouse_input_func(handler, [](aphys::InputHandler& input_handler,
                                          int button, int action) {
    std::cout << "mouse button event: " << button << " " << action << std::endl;
  });
  aphys::add_key_input_func(
      handler, [](aphys::InputHandler& input_handler, int key, int action) {
        std::cout << "key event: " << key << " " << action << std::endl;
      });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    aphys::set_background_RGB({244, 244, 244});

    aphys::set_points_data(points, v_p, v_color);

    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}