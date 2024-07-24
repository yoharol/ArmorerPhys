#include <iostream>
#include <Eigen/Core>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/fem.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

int main() {
  GLFWwindow* window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                           "Example11: 2D Math Plot with Bezier and B-Spline");
  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, -1.0f, 1.0f, -1.0f, 1.0f);

  int controls = 4;
  aphys::MatxXd shape_ref(controls, 2);
  shape_ref << -0.4, 0.3, -0.3, 0.4, -0.1, 0.4, 0.0, 0.3;
  aphys::Matx4i edges(1, 4);
  edges << 0, 1, 2, 3;

  aphys::MatxXd shape1 = shape_ref;
  aphys::MatxXd shape2 = shape_ref;
  aphys::MatxXd ref_point(2, 2);
  ref_point << -0.2, -0.3, 0.0, 0.0;

  aphys::Mat3d rot = aphys::rotate_and_scale_around_center(
      ref_point.row(0), {1.5, 1.5}, -M_PI / 3.0);
  for (int i = 0; i < shape_ref.rows(); i++) {
    aphys::Vec3d homo(shape_ref(i, 0), shape_ref(i, 1), 1.0);
    aphys::Vec3d res = rot * homo;
    shape2.row(i) = res.head(2).transpose();
  }

  int n_samples = 10;
  aphys::Vecxd interpolates = aphys::Vecxd::LinSpaced(n_samples, 0.0, 1.0);
  aphys::MatxXd B(n_samples, controls);
  for (int i = 0; i < n_samples; i++) {
    aphys::Vec4d bparam = aphys::bezier_params(interpolates(i));
    B.row(i) = bparam.transpose();
  }
  aphys::MatxXd pos_samples(n_samples, 2);
  pos_samples = B * shape_ref;
  aphys::MatxXd pos_samp_ref = pos_samples;

  aphys::MatxXd basis(n_samples, 6);
  aphys::MatxXd linear_basis(n_samples, 3);
  for (int i = 0; i < n_samples; i++) {
    aphys::Vec2d v = (pos_samp_ref.row(i) - ref_point.row(0)).transpose();
    basis.row(i) = aphys::quadratic_basis(v).transpose();
    linear_basis.row(i) << v(0), v(1), 1.0;
  }
  aphys::MatxXd T(6, 2);
  aphys::MatxXd A(3, 2);
  {
    T.setZero();
    double angle = -M_PI / 3.0;
    aphys::Mat2d rot;
    rot << cos(angle), -sin(angle), sin(angle), cos(angle);
    aphys::Mat2d scale;
    scale << 1.5, 0.0, 0.0, 1.5;
    T.block(0, 0, 2, 2) = (rot * scale).transpose();
    T.block(5, 0, 1, 2) = ref_point.row(0);
  }
  aphys::MatxXd pos_rig(n_samples, 2);
  aphys::MatxXd pos_target(n_samples, 2);
  pos_rig = linear_basis * A;
  pos_target = B * shape2;

  aphys::BezierPeices bp1(shape1, edges);
  aphys::add_render_func(scene, aphys::get_render_func(bp1));
  aphys::BezierPeices bp2(shape2, edges);
  aphys::add_render_func(scene, aphys::get_render_func(bp2));

  aphys::Points sample_points = aphys::create_points();
  sample_points.color = aphys::RGB{0, 0, 0};
  sample_points.point_size = 4.0f;
  aphys::set_points_data(sample_points, pos_samples.cast<float>(),
                         aphys::MatxXf());
  aphys::add_render_func(scene, aphys::get_render_func(sample_points));

  aphys::Points rig_points = aphys::create_points();
  rig_points.color = aphys::RGB{0, 0, 255};
  rig_points.point_size = 4.0f;
  aphys::set_points_data(rig_points, pos_rig.cast<float>(), aphys::MatxXf());
  aphys::add_render_func(scene, aphys::get_render_func(rig_points));

  aphys::Points origin_points = aphys::create_points();
  origin_points.color = aphys::RGB{255, 0, 0};
  origin_points.point_size = 8.0f;
  aphys::set_points_data(origin_points, ref_point.cast<float>(),
                         aphys::MatxXf());
  aphys::add_render_func(scene, aphys::get_render_func(origin_points));

  auto set_target = [&]() {
    pos_target = B * shape2;
    aphys::MatxXd PTP = basis.transpose() * basis;
    aphys::MatxXd rhs = basis.transpose() * pos_target;
    T = PTP.ldlt().solve(rhs);

    pos_rig = basis * T;
    ref_point.row(1) = T.bottomRows(1);
    aphys::set_points_data(rig_points, pos_rig.cast<float>(), aphys::MatxXf());
    aphys::set_points_data(origin_points, ref_point.cast<float>(),
                           aphys::MatxXf());
  };
  set_target();

  int curr_idx = 0;
  aphys::InputHandler& handler = aphys::create_input_handler(window);
  aphys::add_mouse_move_func(handler, [&](aphys::InputHandler& handler) {
    if (handler.left_pressing) {
      float x = handler.xpos;
      float y = handler.ypos;
      aphys::camera2d_screen_to_world(scene.camera, x, y);
      aphys::Vecxd p(2);
      p << x, y;
      curr_idx = aphys::find_nearest_point(shape2, p);
      shape2.row(curr_idx) = p.transpose();
      bp2.set_data(shape2, edges);
      set_target();
    }
  });

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
