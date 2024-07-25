#include <iostream>
#include <Eigen/Core>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/fem.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 1000;

int main() {
  GLFWwindow* window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                           "Example11: 2D Math Plot with Bezier and B-Spline");
  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, -1.0f, 1.0f, -0.5f, 1.5f);
  aphys::Gui gui = aphys::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 50;

  aphys::MatxXd shape_ref(12, 2);
  shape_ref << 0.4, 0.2, 0.45, 0.15, 0.55, 0.15, 0.6, 0.2, 0.35, 0.35, 0.35,
      0.65, 0.4, 0.8, 0.45, 0.85, 0.55, 0.85, 0.6, 0.8, 0.65, 0.35, 0.65, 0.65;
  aphys::Matx4i edges(4, 4);
  edges << 0, 1, 2, 3, 0, 4, 5, 6, 6, 7, 8, 9, 3, 10, 11, 9;

  aphys::MatxXd shape1 = shape_ref;
  aphys::Mat3d rot = aphys::rotate_around_center({0.5, 0.4}, -M_PI / 3.0);
  for (int i : {5, 6, 7, 8, 9, 11}) {
    aphys::Vec3d homo(shape_ref(i, 0), shape_ref(i, 1), 1.0);
    aphys::Vec3d res = rot * homo;
    shape1.row(i) = res.head(2).transpose();
  }
  aphys::MatxXd ref_point1(1, 2);
  ref_point1.row(0) = shape1.colwise().mean();
  // ref_point1 << 0.0, 0.0;

  aphys::MatxXd shape2 = shape_ref;
  aphys::Mat3d rot2 = aphys::rotate_around_center({0.5, 0.4}, M_PI / 3.0);
  for (int i : {5, 6, 7, 8, 9, 11}) {
    aphys::Vec3d homo(shape_ref(i, 0), shape_ref(i, 1), 1.0);
    aphys::Vec3d res = rot2 * homo;
    shape2.row(i) = res.head(2).transpose();
  }
  shape2.rowwise() -= aphys::Vec2d(1.0, 0.0).transpose();
  aphys::MatxXd ref_point2(1, 2);
  ref_point2.row(0) = shape2.colwise().mean();
  // ref_point2 << 0.8, 0.8;

  int n_samples = 8;
  aphys::MatxXd B(n_samples * edges.rows(), shape_ref.rows());
  aphys::Matx2i sample_edges((n_samples - 1) * edges.rows(), 2);
  B.setZero();
  for (int i = 0; i < edges.rows(); i++) {
    aphys::Vecxd param = aphys::Vecxd::LinSpaced(n_samples, 0.0, 1.0);
    for (int j = 0; j < n_samples; j++) {
      int idx = i * n_samples + j;
      aphys::Vec4d params = aphys::bezier_params(param(j));
      B(idx, edges(i, 0)) = params(0);
      B(idx, edges(i, 1)) = params(1);
      B(idx, edges(i, 2)) = params(2);
      B(idx, edges(i, 3)) = params(3);
      if (j < n_samples - 1) {
        sample_edges.row(i * (n_samples - 1) + j) << idx, idx + 1;
      }
    }
  }
  aphys::MatxXd samle_pos = B * shape2;
  aphys::MatxXd samples1 = B * shape1;
  aphys::MatxXd samples2 = B * shape2;

  int n_terms = edges.rows() * 5 + 1;
  int n_linear_terms = edges.rows() * 2 + 1;
  aphys::MatxXd basis1(n_samples * edges.rows(), n_terms);
  aphys::MatxXd basis2(n_samples * edges.rows(), n_terms);
  aphys::MatxXd linear_basis1(n_samples * edges.rows(), n_linear_terms);
  aphys::MatxXd linear_basis2(n_samples * edges.rows(), n_linear_terms);
  basis1.setZero();
  basis2.setZero();
  linear_basis1.setZero();
  linear_basis2.setZero();
  aphys::MatxXd T1(n_terms, 2);
  aphys::MatxXd T2(n_terms, 2);
  aphys::MatxXd linear_T1(n_linear_terms, 2);
  aphys::MatxXd linear_T2(n_linear_terms, 2);
  aphys::MatxXd D1, D2;
  aphys::MatxXd linear_D1, linear_D2;

  auto reset_basis = [&]() {
    aphys::MatxXd samples1 = B * shape1;
    aphys::MatxXd samples2 = B * shape2;
    for (int i = 0; i < edges.rows(); i++) {
      for (int j = 0; j < n_samples; j++) {
        int idx = i * n_samples + j;
        aphys::Vec2d v = (samples1.row(idx) - ref_point1.row(0)).transpose();
        aphys::Vecxd qb = aphys::quadratic_basis(v);
        for (int k = 0; k < 5; k++) {
          basis1(idx, i * 5 + k) = qb(k);
        }
        qb = v;
        for (int k = 0; k < 2; k++) {
          linear_basis1(idx, i * 2 + k) = qb(k);
        }
        basis1(idx, n_terms - 1) = 1.0;
        linear_basis1(idx, n_linear_terms - 1) = 1.0;

        v = (samples2.row(idx) - ref_point2.row(0)).transpose();
        qb = aphys::quadratic_basis(v);
        for (int k = 0; k < 5; k++) {
          basis2(idx, i * 5 + k) = qb(k);
        }
        qb = v;
        for (int k = 0; k < 2; k++) {
          linear_basis2(idx, i * 2 + k) = qb(k);
        }
        basis2(idx, n_terms - 1) = 1.0;
        linear_basis2(idx, n_linear_terms - 1) = 1.0;
      }
    }

    D1 = (basis1.transpose() * basis1).inverse() * basis1.transpose() * B;
    D2 = (basis2.transpose() * basis2).inverse() * basis2.transpose() * B;
    linear_D1 = (linear_basis1.transpose() * linear_basis1).inverse() *
                linear_basis1.transpose() * B;
    linear_D2 = (linear_basis2.transpose() * linear_basis2).inverse() *
                linear_basis2.transpose() * B;
  };
  reset_basis();

  aphys::MatxXd shape_target = shape_ref;
  double interpolate_t = 0.5;

  auto Initialize = [&]() {
    shape_target = (1 - interpolate_t) * shape1 + interpolate_t * shape2;
    samle_pos = B * shape_target;
  };
  Initialize();

  auto LocalCompute = [&]() {
    linear_T1 = linear_D1 * shape_target;
    linear_T2 = linear_D2 * shape_target;
    T1 = D1 * shape_target;
    T2 = D2 * shape_target;
  };
  LocalCompute();

  auto ExtractRotation = [&](aphys::MatxXd& inputT,
                             aphys::MatxXd& inputLinearT) -> aphys::MatxXd {
    aphys::MatxXd resT = inputT;
    resT.setZero();
    for (int i = 0; i < edges.rows(); i++) {
      aphys::MatxXd rot = inputLinearT.block(i * 2, 0, 2, 2).transpose();
      aphys::MatxXd extracted_rot = aphys::rotation_extraction<2>(rot);
      resT.block(i * 5, 0, 2, 2) = extracted_rot.transpose();
    }
    resT.bottomRows(1) = inputT.bottomRows(1);
    return resT;
  };

  int mode = 0;
  auto OneStepOptimize = [&]() {
    aphys::MatxXd R1 = ExtractRotation(T1, linear_T1);
    aphys::MatxXd R2 = ExtractRotation(T2, linear_T2);
    double t = interpolate_t;
    t = 0.0;
    aphys::MatxXd lhs =
        (1.0 - t) * (D1.transpose() * D1) + t * (D2.transpose() * D2);
    aphys::MatxXd rhs =
        (1.0 - t) * (D1.transpose() * R1) + t * (D2.transpose() * R2);
    aphys::MatxXd newq = lhs.ldlt().solve(rhs);
    t = 1.0;
    lhs = (1.0 - t) * (D1.transpose() * D1) + t * (D2.transpose() * D2);
    rhs = (1.0 - t) * (D1.transpose() * R1) + t * (D2.transpose() * R2);
    aphys::MatxXd newq2 = lhs.ldlt().solve(rhs);
    if (mode == 0) {
      t = interpolate_t;
    } else if (mode == 1) {
      t = 0.0;
    } else if (mode == 2) {
      t = 1.0;
    }
    aphys::MatxXd resq = (1.0 - t) * newq + t * newq2;
    shape_target = resq;

    // samle_pos = B * shape_target;
  };

  aphys::BezierPeices bp1(shape1, edges);
  aphys::BezierPeices bp2(shape2, edges);
  aphys::BezierPeices bp_result(shape1, edges);
  bp1.set_line_color(aphys::RGB{0, 0, 0}, 0.3);
  bp2.set_line_color(aphys::RGB{0, 0, 0}, 0.3);
  bp1.set_handler_edges_alpha(0.2);
  bp2.set_handler_edges_alpha(0.2);
  bp_result.set_handler_edges_alpha(0.2);
  bp1.set_handler_size(3.0);
  bp2.set_handler_size(3.0);
  bp_result.set_handler_size(3.0);
  bp_result.set_line_width(2.0);
  aphys::Points refp1 = aphys::create_points();
  refp1.color = aphys::RGB{0, 0, 255};
  aphys::set_points_data(refp1, ref_point1.cast<float>(), aphys::MatxXf());
  aphys::Points refp2 = aphys::create_points();
  refp2.color = aphys::RGB(0, 0, 255);
  aphys::set_points_data(refp2, ref_point2.cast<float>(), aphys::MatxXf());

  aphys::Edges sampleedge = aphys::create_edges();
  sampleedge.color = aphys::RGB{0, 133, 198};
  sampleedge.width = 1.0f;
  aphys::set_edges_data(sampleedge, samle_pos.cast<float>(), sample_edges,
                        aphys::MatxXf());

  aphys::add_render_func(scene, aphys::get_render_func(bp1));
  aphys::add_render_func(scene, aphys::get_render_func(bp2));
  aphys::add_render_func(scene, aphys::get_render_func(bp_result));
  aphys::add_render_func(scene, aphys::get_render_func(refp1));
  aphys::add_render_func(scene, aphys::get_render_func(refp2));
  aphys::add_render_func(scene, aphys::get_render_func(sampleedge));

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
      bp_result.set_data(shape_target, edges);
      aphys::set_edges_data(sampleedge, samle_pos.cast<float>(), sample_edges,
                            aphys::MatxXf());
    }
  });

  float input_w = 0.0f;
  aphys::add_gui_func(gui, [&]() {
    ImGui::SliderFloat("Input W", &input_w, 0.0, 1.0);
    if (input_w != interpolate_t) {
      interpolate_t = input_w;
      Initialize();
      LocalCompute();
      OneStepOptimize();
      bp_result.set_data(shape_target, edges);
      aphys::set_edges_data(sampleedge, samle_pos.cast<float>(), sample_edges,
                            aphys::MatxXf());
    }
    ImGui::RadioButton("Interpolate", &mode, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Shape 0", &mode, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Shape 1", &mode, 2);
  });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();
    aphys::handle_gui_input(gui);

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);
    aphys::render_gui(gui);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
