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
  aphys::Gui gui = aphys::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 50;

  int controls = 4;
  aphys::MatxXd shape_ref(controls, 2);
  shape_ref << -0.4, 0.3, -0.3, 0.4, -0.1, 0.4, 0.0, 0.3;
  aphys::Matx4i edges(1, 4);
  edges << 0, 1, 2, 3;

  aphys::MatxXd shape1 = shape_ref;
  aphys::MatxXd shape2 = shape_ref;
  aphys::MatxXd shape_target = shape_ref;
  aphys::MatxXd ref_point(4, 2);
  ref_point << -0.2, 0.0, 0.1, -0.6, 0.0, 0.0, 0.0, 0.0;

  aphys::Mat3d rot = aphys::rotate_and_scale_around_center(
      {-0.2, -0.3}, {1.5, 1.5}, -M_PI / 1.5);
  for (int i = 0; i < shape_ref.rows(); i++) {
    aphys::Vec3d homo(shape_ref(i, 0), shape_ref(i, 1), 1.0);
    aphys::Vec3d res = rot * homo;
    shape2.row(i) = res.head(2).transpose();
  }

  double interpolate_t = 0.5;

  int n_samples = 10;
  aphys::Vecxd interpolates = aphys::Vecxd::LinSpaced(n_samples, 0.0, 1.0);
  aphys::MatxXd B(n_samples, controls);
  for (int i = 0; i < n_samples; i++) {
    aphys::Vec4d bparam = aphys::bezier_params(interpolates(i));
    B.row(i) = bparam.transpose();
  }
  aphys::MatxXd pos_samples1(n_samples, 2);
  aphys::MatxXd pos_samples2(n_samples, 2);
  aphys::MatxXd pos_sample_raw(n_samples, 2);

  aphys::MatxXd basis1(n_samples, 6);
  aphys::MatxXd basis2(n_samples, 6);
  aphys::MatxXd T1(6, 2);
  aphys::MatxXd T2(6, 2);
  aphys::MatxXd D1;
  aphys::MatxXd V1;
  aphys::MatxXd D2;
  aphys::MatxXd V2;

  auto reset_basis = [&]() {
    pos_samples1 = B * shape1;
    pos_samples2 = B * shape2;
    for (int i = 0; i < n_samples; i++) {
      aphys::Vec2d v = (pos_samples1.row(i) - ref_point.row(0)).transpose();
      basis1.row(i) = aphys::quadratic_basis(v).transpose();
      v = (pos_samples2.row(i) - ref_point.row(1)).transpose();
      basis2.row(i) = aphys::quadratic_basis(v).transpose();
    }
    D1 = (basis1.transpose() * basis1).inverse() * basis1.transpose() * B;
    V1 = D1.bottomRows(1);
    D2 = (basis2.transpose() * basis2).inverse() * basis2.transpose() * B;
    V2 = D2.bottomRows(1);
  };
  reset_basis();

  aphys::MatxXd pos_target(n_samples, 2);
  pos_target = B * shape2;

  aphys::BezierPeices bp1(shape1, edges);
  aphys::add_render_func(scene, aphys::get_render_func(bp1));
  aphys::BezierPeices bp2(shape2, edges);
  aphys::add_render_func(scene, aphys::get_render_func(bp2));
  aphys::BezierPeices bp_raw(shape_target, edges);
  aphys::add_render_func(scene, aphys::get_render_func(bp_raw));

  aphys::Points sample_points = aphys::create_points();
  sample_points.color = aphys::RGB{0, 0, 0};
  sample_points.point_size = 4.0f;
  aphys::set_points_data(sample_points, pos_samples1.cast<float>(),
                         aphys::MatxXf());
  aphys::add_render_func(scene, aphys::get_render_func(sample_points));

  // aphys::add_render_func(scene, aphys::get_render_func(rig_points));

  aphys::Points origin_points = aphys::create_points();
  aphys::MatxXf origin_points_color(4, 3);
  origin_points_color << 0.0, 0.0, 1.0,  //
      0.0, 1.0, 0.0,                     //
      0.0, 0.0, 1.0,                     //
      0.0, 1.0, 0.0;
  origin_points.color = aphys::RGB{255, 0, 0};
  origin_points.point_size = 3.0f;
  aphys::set_points_data(origin_points, ref_point.cast<float>(),
                         origin_points_color);
  aphys::add_render_func(scene, aphys::get_render_func(origin_points));

  auto Initialize = [&]() {
    shape_target = (1 - interpolate_t) * shape1 + interpolate_t * shape2;
    bp_raw.set_data(shape_target, edges);
    pos_sample_raw = B * shape_target;
    aphys::set_points_data(sample_points, pos_sample_raw.cast<float>(),
                           aphys::MatxXf());
  };
  Initialize();

  auto LocalCompute = [&]() {
    T1 = D1 * shape_target;
    T2 = D2 * shape_target;
    ref_point.row(2) = T1.bottomRows(1);
    ref_point.row(3) = T2.bottomRows(1);
  };
  LocalCompute();

  auto ExtractRotation = [](aphys::MatxXd& inputT) -> aphys::MatxXd {
    aphys::MatxXd resT = inputT;
    resT.setZero();
    aphys::MatxXd rot = inputT.block(0, 0, 2, 2).transpose();
    aphys::MatxXd extracted_rot = aphys::rotation_extraction<2>(rot);
    resT.block(0, 0, 2, 2) = extracted_rot.transpose();
    resT.block(5, 0, 1, 2) = inputT.bottomRows(1);
    return resT;
  };

  auto OneStepOptimize = [&]() {
    aphys::MatxXd R1 = ExtractRotation(T1);
    aphys::MatxXd R2 = ExtractRotation(T2);
    double t = interpolate_t;
    t = 0.0;
    aphys::MatxXd lhs =
        (1.0 - t) * (D1.transpose() * D1) + t * (D2.transpose() * D2);
    aphys::MatxXd rhs =
        (1.0 - t) * (D1.transpose() * R1) + t * (D2.transpose() * R2);
    aphys::MatxXd newq1 = lhs.ldlt().solve(rhs);
    t = 1.0;
    lhs = (1.0 - t) * (D1.transpose() * D1) + t * (D2.transpose() * D2);
    rhs = (1.0 - t) * (D1.transpose() * R1) + t * (D2.transpose() * R2);
    aphys::MatxXd newq2 = lhs.ldlt().solve(rhs);
    t = interpolate_t;
    aphys::MatxXd resq = (1.0 - t) * newq1 + t * newq2;
    shape_target = resq;
    bp_raw.set_data(shape_target, edges);
    LocalCompute();
  };

  int curr_idx = 0;
  aphys::add_gui_mouse_input_func(gui, [&]() {
    if (ImGui::IsMouseDragging(0)) {
      float x = ImGui::GetMousePos().x / SCR_WIDTH;
      float y = 1.0 - ImGui::GetMousePos().y / SCR_HEIGHT;
      aphys::camera2d_screen_to_world(scene.camera, x, y);
      aphys::Vecxd p(2);
      p << x, y;
      curr_idx = aphys::find_nearest_point(shape2, p);
      shape2.row(curr_idx) = p.transpose();
      bp2.set_data(shape2, edges);
      reset_basis();
      Initialize();
      LocalCompute();
      OneStepOptimize();
    }
  });

  float input_t = 0.0;
  aphys::add_gui_func(gui, [&]() {
    ImGui::SliderFloat("weight", &input_t, 0.0, 1.0);
    if (input_t != interpolate_t) {
      interpolate_t = input_t;
      Initialize();
      LocalCompute();
      OneStepOptimize();
    }
    if (ImGui::Button("Optimize")) {
      OneStepOptimize();
    }
  });

  aphys::Matx2i edges_idx(2, 2);
  edges_idx << 0, 1, 0, 2;
  aphys::Edges edges1 = aphys::create_edges();
  edges1.alpha = 0.5f;
  aphys::MatxXd tr1(3, 2);
  aphys::Edges edges2 = aphys::create_edges();
  edges2.alpha = 0.5f;
  aphys::MatxXd tr2(3, 2);
  aphys::Edges edges3 = aphys::create_edges();
  edges3.alpha = 0.5f;
  aphys::MatxXd tr3(3, 2);
  aphys::Edges edges4 = aphys::create_edges();
  edges4.alpha = 0.5f;
  aphys::MatxXd tr4(3, 2);
  aphys::add_render_func(scene, aphys::get_render_func(edges1));
  aphys::add_render_func(scene, aphys::get_render_func(edges2));
  aphys::add_render_func(scene, aphys::get_render_func(edges3));
  aphys::add_render_func(scene, aphys::get_render_func(edges4));

  auto set_tris = [&]() {
    tr1 << ref_point.row(0), shape1.row(0), shape1.row(3);
    tr2 << ref_point.row(1), shape2.row(0), shape2.row(3);
    tr3 << ref_point.row(2), shape_target.row(0), shape_target.row(3);
    tr4 << ref_point.row(3), shape_target.row(0), shape_target.row(3);

    aphys::set_edges_data(edges1, tr1.cast<float>(), edges_idx,
                          aphys::MatxXf());
    aphys::set_edges_data(edges2, tr2.cast<float>(), edges_idx,
                          aphys::MatxXf());
    aphys::set_edges_data(edges3, tr3.cast<float>(), edges_idx,
                          aphys::MatxXf());
    aphys::set_edges_data(edges4, tr4.cast<float>(), edges_idx,
                          aphys::MatxXf());
  };

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();
    aphys::handle_gui_input(gui);

    aphys::set_points_data(origin_points, ref_point.cast<float>(),
                           origin_points_color);
    set_tris();

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);
    aphys::render_gui(gui);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
