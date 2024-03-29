#include <iostream>
#include <Eigen/Core>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

int main() {
  GLFWwindow* window = aphys::create_window(
      SCR_WIDTH, SCR_HEIGHT, "Example9: 2D Math Plot with Projective Dynamics");
  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, 0.0f, 1.0f, 0.0f, 1.0f);

  aphys::BezierSpline spline = aphys::create_bezier_spline();
  aphys::MatxXf& control_points = spline.control_points;
  aphys::MatxXf& deriv_handles = spline.deriv_handles;
  control_points.resize(4, 2);
  control_points << 0.0f, 0.5f, 0.2f, 0.3f, 0.5f, 0.7f, 1.0f, 0.4f;
  deriv_handles.resize(4, 2);
  deriv_handles << 0.1f, 0.1f, 0.2f, -0.05f, -0.13f, 0.0f, 0.0f, 0.2f;
  aphys::set_bezier_spline_data(
      20, spline, control_points, deriv_handles, aphys::RGB(1.0f, 0.0f, 0.0f),
      aphys::RGB(0.0f, 1.0f, 0.0f), aphys::RGB(0.0f, 0.0f, 1.0f));
  aphys::add_render_func(scene, aphys::get_render_func(spline));

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}