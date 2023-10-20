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

  armgl::InputHandler& handler = armgl::create_input_handler(window);
  armgl::add_mouse_move_func(handler, [&](armgl::InputHandler& input_handler) {
    if (input_handler.left_pressing) {
      float xpos, ypos;
      xpos = handler.xpos;
      ypos = handler.ypos;
      armgl::camera2d_screen_to_world(scene.camera, xpos, ypos);
      armgl::Vec2f p(xpos, ypos);
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

    armgl::set_points_data(points, v_p, armgl::MatxXf());
    armgl::set_ruler2d_data(ruler, v_p.row(0), v_p.row(1), 4.f);

    armgl::set_background_RGB({244, 244, 244});
    armgl::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}