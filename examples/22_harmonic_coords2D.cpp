#include <Eigen/Core>
#include <iostream>
#include <random>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/fem.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

void ClosestPointOnBezier(const aphys::Vecxd& refp, const aphys::MatxXd& verts,
                          const aphys::Matx4i& edges, int& cidx, double& ct) {
  double minDis = 1e10;
  cidx = -1;
  for (int i = 0; i < edges.rows(); i++) {
    aphys::Vecxd p0 = verts.row(edges(i, 0));
    aphys::Vecxd p1 = verts.row(edges(i, 1));
    aphys::Vecxd p2 = verts.row(edges(i, 2));
    aphys::Vecxd p3 = verts.row(edges(i, 3));

    int N = 40;
    for (int j = 0; j <= N; j++) {
      float t = (float)j / N;
      aphys::Vecxd p = aphys::bezier_interpolate(p0, p1, p2, p3, t);
      double dis = (p - refp).norm();
      if (dis < minDis) {
        minDis = dis;
        cidx = i;
        ct = t;
      }
    }
  }
}

int main() {
  GLFWwindow* window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                           "Example11: 2D Math Plot with Bezier and B-Spline");
  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, 0.0f, 1.0f, 0.0f, 1.0f);
  aphys::Gui gui = aphys::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 50;

  aphys::MatxXd shape_ref(12, 2);
  shape_ref << 0.4, 0.2, 0.45, 0.15, 0.55, 0.15, 0.6, 0.2, 0.35, 0.35, 0.35,
      0.65, 0.4, 0.8, 0.45, 0.85, 0.55, 0.85, 0.6, 0.8, 0.65, 0.35, 0.65, 0.65;
  aphys::Matx4i edges(4, 4);
  edges << 0, 1, 2, 3,  //
      3, 10, 11, 9,     //
      9, 8, 7, 6,       //
      6, 5, 4, 0;
  aphys::MatxXd shape = shape_ref;

  /*aphys::Vecxd ltp(2);
  ltp << 0.4, 0.75;
  aphys::Vecxd rbp(2);
  rbp << 0.6, 0.2;
  int NX = 8;
  int NY = 16;
  aphys::MatxXd ref_points(NX * NY, 2);
  for (int i = 0; i < NX; i++)
    for (int j = 0; j < NY; j++) {
      double x = ltp(0) + (rbp(0) - ltp(0)) * i / (NX - 1);
      double y = ltp(1) + (rbp(1) - ltp(1)) * j / (NY - 1);
      ref_points.row(i * NY + j) << x, y;
    }*/

  int NB = 15;
  int NInterp = 15;
  aphys::MatxXd ref_points(NB * NInterp * edges.rows(), 2);
  aphys::MatxXd sampled_ref_points = ref_points;
  aphys::Vecxd avgP = shape_ref.colwise().mean();
  aphys::Matx2i sample_edges(NInterp * (NB * edges.rows()), 2);
  for (int i = 0; i < NB; i++) {
    double t = (double)i / NB;
    for (int k = 0; k < edges.rows(); k++) {
      aphys::Vecxd p0 = shape_ref.row(edges(k, 0));
      aphys::Vecxd p1 = shape_ref.row(edges(k, 1));
      aphys::Vecxd p2 = shape_ref.row(edges(k, 2));
      aphys::Vecxd p3 = shape_ref.row(edges(k, 3));
      aphys::Vecxd p = aphys::bezier_interpolate(p0, p1, p2, p3, t);
      for (int j = 0; j < NInterp; j++) {
        double t2 = 1.0 - (double)j / NInterp;
        ref_points.row(i + k * NB + j * NB * edges.rows()) =
            (t2 * p + (1.0 - t2) * avgP).transpose();
      }
    }
  }

  int edge_count = 0;
  for (int j = 0; j < NInterp; j++) {
    int previdx = -1;
    int startidx;
    for (int k = 0; k < edges.rows(); k++)
      for (int i = 0; i < NB; i++) {
        int idx = i + k * NB + j * NB * edges.rows();
        if (previdx != -1) {
          sample_edges.row(edge_count) << previdx, idx;
          edge_count++;
        } else {
          startidx = idx;
        }
        previdx = idx;
      }
    sample_edges.row(edge_count) << previdx, startidx;
    edge_count++;
  }

  aphys::BezierPeices bp(shape_ref, edges);
  aphys::add_render_func(scene, aphys::get_render_func(bp));

  /*aphys::Points points = aphys::create_points();
  points.point_size = 2.5;
  points.color = {0, 0, 0};
  aphys::set_points_data(points, ref_points.cast<float>(), aphys::MatxXf());
  aphys::add_render_func(scene, aphys::get_render_func(points));*/

  aphys::Edges draw_edges = aphys::create_edges();
  aphys::set_edges_data(draw_edges, ref_points.cast<float>(), sample_edges,
                        aphys::MatxXf());
  aphys::add_render_func(scene, aphys::get_render_func(draw_edges));

  int curr_idx = 0;
  aphys::add_gui_mouse_input_func(gui, [&]() {
    float x = ImGui::GetMousePos().x / SCR_WIDTH;
    float y = 1.0 - ImGui::GetMousePos().y / SCR_HEIGHT;
    aphys::camera2d_screen_to_world(scene.camera, x, y);
    aphys::Vecxd p(2);
    p << x, y;
    if (ImGui::IsMouseClicked(0)) {
      curr_idx = aphys::find_nearest_point(shape, p);
    }
    if (ImGui::IsMouseDragging(0)) {
      shape.row(curr_idx) = p.transpose();
      bp.set_data(shape, edges);
    }
  });

  glfwSwapInterval(1);

  int n_points = ref_points.rows();
  int n_controls = shape_ref.rows();
  int n_peices = edges.rows();

  aphys::MatxXd weights(n_points, n_controls);
  auto weight_idx = [&n_controls](int i, int j) { return i * n_controls + j; };
  double* weights_data_array = weights.data();
  weights.setZero();
  aphys::MatxXd weights_avged = weights;

  const double eps = 1e-5;
  int nWalks = 0;
  const int maxSteps = 16;

  auto WalkOnSphereSolver = [&]() {
#pragma omp parallel for reduction(+ : weights_data_array[ : weights.size()])
    for (int i = 0; i < ref_points.rows(); i++) {
      // for (int k = 0; k < _nWalks; k++) {
      aphys::Vecxd currp = ref_points.row(i);
      double R = 1e10;
      int steps = 0;
      int cidx = -1;
      double ct = 0;
      while (R > eps && steps < maxSteps) {
        ClosestPointOnBezier(currp, shape_ref, edges, cidx, ct);
        aphys::Vecxd p0 = shape_ref.row(edges(cidx, 0));
        aphys::Vecxd p1 = shape_ref.row(edges(cidx, 1));
        aphys::Vecxd p2 = shape_ref.row(edges(cidx, 2));
        aphys::Vecxd p3 = shape_ref.row(edges(cidx, 3));
        aphys::Vecxd p = aphys::bezier_interpolate(p0, p1, p2, p3, ct);
        R = (p - currp).norm();
        double theta = aphys::random(0., 2. * M_PI);
        currp(0) += R * cos(theta);
        currp(1) += R * sin(theta);
        steps++;
      }

      aphys::Vec4d params = aphys::bezier_params(ct);
      weights_data_array[weight_idx(i, edges(cidx, 0))] += params(0);
      weights_data_array[weight_idx(i, edges(cidx, 1))] += params(1);
      weights_data_array[weight_idx(i, edges(cidx, 2))] += params(2);
      weights_data_array[weight_idx(i, edges(cidx, 3))] += params(3);
      // }
    }

    nWalks++;

    // weights /= _nWalks;
  };

  aphys::Matx3f weight_color(n_points, 3);

  aphys::add_gui_func(gui,
                      [&]() { ImGui::Text(std::to_string(nWalks).c_str()); });

  // for (int i = 0; i < weights.rows(); i++) {
  //   weights.row(i) /= weights.row(i).sum();
  // }

  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();
    aphys::handle_gui_input(gui);

    for (int i = 0; i < 10; i++) WalkOnSphereSolver();

    weights_avged = weights / nWalks;

    for (int i = 0; i < n_points; i++) {
      aphys::Vecxd cp(2);
      cp.setZero();
      for (int j = 0; j < n_controls; j++) {
        cp += weights_avged(i, j) * shape.row(j);
      }
      sampled_ref_points.row(i) = cp.transpose();
    }

    for (int i = 0; i < n_points; i++) {
      aphys::Vec3d color = aphys::heat_rgb(weights(i, curr_idx), 0.0, 1.0);
      weight_color.row(i) = color.cast<float>();
    }
    // aphys::set_points_data(points, sampled_ref_points.cast<float>(),
    //                        weight_color);
    aphys::set_edges_data(draw_edges, sampled_ref_points.cast<float>(),
                          sample_edges, aphys::MatxXf());

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);
    aphys::render_gui(gui);
    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
