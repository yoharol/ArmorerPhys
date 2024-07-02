#include <iostream>
#include <Eigen/Core>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

int main() {
  GLFWwindow* window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                           "Example11: 2D Math Plot with Bezier and B-Spline");
  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, 0.0f, 1.0f, 0.0f, 1.0f);

  // bezier spline
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

  // b spline
  aphys::MatxXd control_points_b;
  control_points_b.resize(5, 2);
  control_points_b << 0.05, 0.05, 0.2, 0.3, 0.5, 0.7, 0.8, 0.5, 0.95, 0.95;

  aphys::Points points_b = aphys::create_points();
  aphys::set_points_data(points_b, control_points_b.cast<float>(),
                         aphys::MatxXf());
  points_b.point_size = 13.0f;
  points_b.color = aphys::RGB(0, 0, 244);
  aphys::add_render_func(scene, aphys::get_render_func(points_b));

  aphys::MatxXd sample_b(1, 2);
  sample_b << 0.0, 0.0;

  aphys::MatxXd sample_deriv_b(2, 2);
  aphys::MatxXd sample_second_deriv_b(2, 2);

  aphys::Points sample_points_b = aphys::create_points();
  aphys::set_points_data(sample_points_b, sample_b.cast<float>(),
                         aphys::MatxXf());
  sample_points_b.point_size = 7.0f;
  sample_points_b.color = aphys::RGB(244, 0, 0);
  aphys::add_render_func(scene, aphys::get_render_func(sample_points_b));

  aphys::Lines sample_deriv_lines_b = aphys::create_lines();
  aphys::set_lines_data(sample_deriv_lines_b, sample_deriv_b.cast<float>(),
                        aphys::MatxXf());
  sample_deriv_lines_b.width = 2.0f;
  sample_deriv_lines_b.color = aphys::RGB(0, 244, 0);
  aphys::add_render_func(scene, aphys::get_render_func(sample_deriv_lines_b));

  aphys::Lines sample_second_deriv_lines_b = aphys::create_lines();
  aphys::set_lines_data(sample_second_deriv_lines_b,
                        sample_second_deriv_b.cast<float>(), aphys::MatxXf());
  sample_second_deriv_lines_b.width = 2.0f;
  sample_second_deriv_lines_b.color = aphys::RGB(122, 122, 0);
  aphys::add_render_func(scene,
                         aphys::get_render_func(sample_second_deriv_lines_b));

  glfwSwapInterval(1);

  int k = 4;

  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();

    float time = scene.time;
    float T = 2.0f;
    time = time - T * floor(time / T);
    double t = time / T;

    int idx;
    aphys::Vecxd basis =
        aphys::Bspline_Basis(t, control_points_b.rows(), k, idx);
    sample_b.row(0) =
        (basis.transpose() * control_points_b.block(idx, 0, k, 2)).transpose();

    aphys::set_points_data(sample_points_b, sample_b.cast<float>(),
                           aphys::MatxXf());

    aphys::Vecxd deriv_basis =
        aphys::Bspline_deriv_Basis(t, control_points_b.rows(), k, idx);
    sample_deriv_b.row(0) = sample_b.row(0);
    sample_deriv_b.row(1) =
        (deriv_basis.transpose() * control_points_b.block(idx, 0, k, 2)) * 0.1 +
        sample_b.row(0);

    aphys::Vecxd second_deriv_basis =
        aphys::Bspline_second_deriv_Basis(t, control_points_b.rows(), k, idx);
    sample_second_deriv_b.row(0) = sample_deriv_b.row(1);
    sample_second_deriv_b.row(1) = (second_deriv_basis.transpose() *
                                    control_points_b.block(idx, 0, k, 2)) *
                                       0.02 +
                                   sample_deriv_b.row(1);

    aphys::set_lines_data(sample_deriv_lines_b, sample_deriv_b.cast<float>(),
                          aphys::MatxXf());
    aphys::set_lines_data(sample_second_deriv_lines_b,
                          sample_second_deriv_b.cast<float>(), aphys::MatxXf());

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}