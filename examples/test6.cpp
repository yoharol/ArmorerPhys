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

  int n_samples = 10;
  aphys::MatxXd B(n_samples * edges.rows(), shape_ref.rows());
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
  aphys::MatxXd D1, D2;
  aphys::MatxXd linear_D1, linear_D2;
  D1 = (basis1.transpose() * basis1).inverse() * basis1.transpose() * B;
  D2 = (basis2.transpose() * basis2).inverse() * basis2.transpose() * B;
  linear_D1 = (linear_basis1.transpose() * linear_basis1).inverse() *
              linear_basis1.transpose() * B;
  linear_D2 = (linear_basis2.transpose() * linear_basis2).inverse() *
              linear_basis2.transpose() * B;

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
  };
  LocalCompute();

  auto ExtractRotation = [&](aphys::MatxXd& inputT) -> aphys::MatxXd {
    aphys::MatxXd resT = inputT;
    resT.setZero();
    for (int i = 0; i < edges.rows(); i++) {
      aphys::MatxXd rot = inputT.block(i * 2, 0, 2, 2).transpose();
      aphys::MatxXd extracted_rot = aphys::rotation_extraction<2>(rot);
      resT.block(i * 2, 0, 2, 2) = rot.transpose();
    }
    resT.bottomRows(1) = inputT.bottomRows(1);
    return resT;
  };

  auto OneStepOptimize = [&]() {
    aphys::MatxXd linear_R1 = ExtractRotation(linear_T1);
    aphys::MatxXd linear_R2 = ExtractRotation(linear_T2);
    aphys::MatxXd R1, R2;
    R1.resize(n_terms, 2);
    R2.resize(n_terms, 2);
    R1.setZero();
    R2.setZero();
    for (int i = 0; i < edges.rows(); i++) {
      R1.block(i * 5, 0, 5, 2) = linear_R1.block(i * 2, 0, 2, 2);
      R2.block(i * 5, 0, 5, 2) = linear_R2.block(i * 2, 0, 2, 2);
    }
    R1.bottomRows(1) = linear_R1.bottomRows(1);
    R2.bottomRows(1) = linear_R2.bottomRows(1);
    double t = interpolate_t;
    aphys::MatxXd lhs = (1.0 - t) * (linear_D1.transpose() * linear_D1) +
                        t * (linear_D2.transpose() * linear_D2);
    aphys::MatxXd rhs = (1.0 - t) * (linear_D1.transpose() * linear_R1) +
                        t * (linear_D2.transpose() * linear_R2);
    aphys::MatxXd newq = lhs.ldlt().solve(rhs);
    samle_pos = linear_basis2 * linear_T2;
    samle_pos.bottomRows(1) = linear_T2.bottomRows(1);
    shape_target = newq;
    // samle_pos = B * shape_target;
  };

  aphys::BezierPeices bp1(shape1, edges);
  aphys::BezierPeices bp2(shape2, edges);
  aphys::BezierPeices bp_result(shape1, edges);
  aphys::Points refp1 = aphys::create_points();
  refp1.color = aphys::RGB{0, 0, 255};
  aphys::set_points_data(refp1, ref_point1.cast<float>(), aphys::MatxXf());
  aphys::Points refp2 = aphys::create_points();
  refp2.color = aphys::RGB(0, 0, 255);
  aphys::set_points_data(refp2, ref_point2.cast<float>(), aphys::MatxXf());

  aphys::Points testpoints = aphys::create_points();
  testpoints.color = aphys::RGB{0, 0, 0};
  testpoints.point_size = 4.0f;
  aphys::add_render_func(scene, aphys::get_render_func(testpoints));

  aphys::add_render_func(scene, aphys::get_render_func(bp1));
  aphys::add_render_func(scene, aphys::get_render_func(bp2));
  aphys::add_render_func(scene, aphys::get_render_func(bp_result));
  aphys::add_render_func(scene, aphys::get_render_func(refp1));
  aphys::add_render_func(scene, aphys::get_render_func(refp2));

  // try laplacian smooth on high order mesh

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();

    double time = glfwGetTime();
    double w = 0.5 + 0.5 * sin(time);
    interpolate_t = w;

    Initialize();
    LocalCompute();
    OneStepOptimize();
    OneStepOptimize();
    OneStepOptimize();
    OneStepOptimize();
    OneStepOptimize();
    OneStepOptimize();
    OneStepOptimize();

    bp_result.set_data(shape_target, edges);

    aphys::set_points_data(testpoints, samle_pos.cast<float>(),
                           aphys::MatxXf());

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
