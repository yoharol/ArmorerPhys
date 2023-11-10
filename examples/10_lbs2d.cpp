#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerSim/RenderCore.h"

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 400;

int main() {
  GLFWwindow* window = asim::create_window(SCR_WIDTH, SCR_HEIGHT,
                                           "Example10 Affine2d Visualization");

  asim::Scene scene =
      asim::create_scene(asim::default_light, asim::default_camera);
  asim::set_2d_camera(scene.camera, -2.0f, 2.0f, -1.0f, 1.0f);

  asim::Matx2f v_p(2, 2);
  v_p << 0.0f, 0.0f,  //
      1.0f, 1.0f;     //

  asim::Points points = asim::create_points();
  points.point_size = 10.0f;
  asim::Lines ruler =
      asim::create_ruler2d(-2.0f, -2.0f, 2.0f, 2.0f, asim::RGB(200, 34, 0));
  ruler.width = 1.5f;
  asim::Lines grids = asim::create_grid_axis2d(-2.0f, 2.0f, -1.0f, 1.0f, 20, 10,
                                               asim::RGB(0, 67, 198));
  grids.alpha = 0.5f;
  asim::Lines axis =
      asim::create_axis2d(-2.0f, 2.0f, -1.0f, 1.0f, asim::RGB(0, 21, 98));
  axis.width = 2.f;

  asim::add_render_func(scene, asim::get_render_func(points));
  asim::add_render_func(scene, asim::get_render_func(ruler));
  asim::add_render_func(scene, asim::get_render_func(axis));
  asim::add_render_func(scene, asim::get_render_func(grids));

  asim::InputHandler& handler = asim::create_input_handler(window);
  asim::add_mouse_move_func(handler, [&](asim::InputHandler& input_handler) {
    if (input_handler.left_pressing) {
      float xpos, ypos;
      xpos = handler.xpos;
      ypos = handler.ypos;
      asim::camera2d_screen_to_world(scene.camera, xpos, ypos);
      asim::Vec2f p(xpos, ypos);
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
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();

    asim::set_points_data(points, v_p, asim::MatxXf());
    asim::set_ruler2d_data(ruler, v_p.row(0), v_p.row(1), 4.f);

    asim::set_background_RGB({244, 244, 244});
    asim::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}