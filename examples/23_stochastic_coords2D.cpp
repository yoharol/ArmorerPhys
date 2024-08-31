#include <Eigen/Core>
#include <iostream>
#include <random>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/fem.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

void ClosestPointOnPoly(const aphys::Vecxd& refp, const aphys::MatxXd& verts,
                        const aphys::Matx2i& edges, int& cidx, double& ct) {
  double minDis = 1e10;
  cidx = -1;
  for (int i = 0; i < edges.rows(); i++) {
    aphys::Vecxd p0 = verts.row(edges(i, 0));
    aphys::Vecxd p1 = verts.row(edges(i, 1));
    aphys::Vecxd p = p1 - p0;
    double t = aphys::clamp((refp - p0).dot(p) / p.dot(p), 0., 1.);
    p = (1 - t) * p0 + t * p1;
    double dis = (p - refp).squaredNorm();
    if (dis < minDis) {
      minDis = dis;
      cidx = i;
      ct = t;
    }
  }
}

void form_polygon(aphys::MatxXd& shape_ref, aphys::Matx2i& edges,
                  const std::vector<aphys::Vec2d>& input_shape) {
  shape_ref.resize(input_shape.size(), 2);
  for (int i = 0; i < input_shape.size(); i++) {
    shape_ref.row(i) = input_shape[i].transpose();
  }
  edges.resize(input_shape.size(), 2);
  for (int i = 0; i < input_shape.size(); i++) {
    edges.row(i) << i, (i + 1) % input_shape.size();
  }
  if (input_shape.size() == 1) edges.row(0) << 0, 0;
}

void sample_inside_grid(const aphys::MatxXd& shape_ref,
                        const aphys::Matx2i& edges,
                        aphys::MatxXd& sample_points,
                        aphys::Matx2i& sample_edges, const double resolution) {
  double min_x = shape_ref.col(0).minCoeff();
  min_x -= resolution / 2.;
  double max_x = shape_ref.col(0).maxCoeff();
  max_x += resolution / 2.;
  double min_y = shape_ref.col(1).minCoeff();
  min_y -= resolution / 2.;
  double max_y = shape_ref.col(1).maxCoeff();
  max_y += resolution / 2.;
  int NX = (max_x - min_x) / resolution;
  int NY = (max_y - min_y) / resolution;

  auto boundary_intersection = [&](aphys::Vecxd a, aphys::Vecxd b) -> bool {
    for (int i = 0; i < edges.rows(); i++) {
      aphys::Vecxd c = shape_ref.row(edges(i, 0));
      aphys::Vecxd d = shape_ref.row(edges(i, 1));
      double t1, t2;
      bool intersected = aphys::segment_intersection2D(a, b, c, d, t1, t2);
      if (intersected) {
        return true;
      }
    }
    return false;
  };

  aphys::Vec2d start_p;
  start_p << min_x, min_y;

  std::vector<aphys::Vec2d> sp;
  std::vector<aphys::Vec2i> se;

  for (int j = 0; j <= NY; j++) {
    aphys::Vec2d p = start_p;
    int prev_idx = -1;
    int winding_number = 0;

    for (int i = 0; i <= NX; i++) {
      aphys::Vec2d a = p;
      aphys::Vec2d b = a;
      b(0) += resolution;
      if (boundary_intersection(a, b)) {
        winding_number = (winding_number + 1) % 2;
      }

      if (winding_number == 1) {
        sp.push_back(b);
        if (prev_idx != -1) {
          se.push_back({prev_idx, sp.size() - 1});
        }
        prev_idx = sp.size() - 1;
      } else {
        prev_idx = -1;
      }
      p = b;
    }
    start_p(1) += resolution;
  }

  sample_points.resize(sp.size(), 2);
  for (int i = 0; i < sp.size(); i++) {
    sample_points.row(i) = sp[i].transpose();
  }
  sample_edges.resize(se.size(), 2);
  for (int i = 0; i < se.size(); i++) {
    sample_edges.row(i) = se[i].transpose();
  }
}

void WalkOnSphereSolver(const aphys::MatxXd& sample_points,
                        const aphys::MatxXd& shape_ref,
                        const aphys::Matx2i& edges, aphys::MatxXd& weights,
                        int curr_idx, int maxSteps, double eps) {
  int n_samples = sample_points.rows();
  int n_controls = shape_ref.rows();
  for (int i = 0; i < n_samples; i++) {
    aphys::Vecxd currp = sample_points.row(i);
    double R = 1e10;
    int steps = 0;
    int cidx = -1;
    double ct = 0;
    while (R > eps && steps < maxSteps) {
      ClosestPointOnPoly(currp, shape_ref, edges, cidx, ct);
      aphys::Vecxd p0 = shape_ref.row(edges(cidx, 0));
      aphys::Vecxd p1 = shape_ref.row(edges(cidx, 1));
      aphys::Vecxd p = aphys::lerp(p0, p1, ct);
      R = (p - currp).norm();
      double theta = aphys::random(0., 2. * M_PI);
      currp(0) += R * cos(theta);
      currp(1) += R * sin(theta);
      steps++;
    }

    weights(i, edges(cidx, 0)) += 1. - ct;
    weights(i, edges(cidx, 1)) += ct;
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
  gui.height = 100;

  aphys::MatxXd shape_ref;
  std::vector<aphys::Vec2d> input_shape;
  aphys::Matx2i edges;
  aphys::MatxXd shape;

  aphys::MatxXd sample_points;
  aphys::Matx2i sample_edges;
  aphys::MatxXd interpolated_points;

  aphys::MatxXd weights;
  aphys::MatxXd weights_avged;
  int nWalks = 0;

  aphys::Points draw_points = aphys::create_points();
  aphys::add_render_func(scene, aphys::get_render_func(draw_points));

  aphys::Edges draw_polygon = aphys::create_edges();
  draw_polygon.color = {34, 151, 153};
  aphys::add_render_func(scene, aphys::get_render_func(draw_polygon));

  aphys::Points draw_sample_points = aphys::create_points();
  draw_sample_points.point_size = 2;
  draw_sample_points.color = {255, 152, 116};
  aphys::add_render_func(scene, aphys::get_render_func(draw_sample_points));

  aphys::Edges draw_sample_edges = aphys::create_edges();
  draw_sample_edges.color = {255, 152, 116};
  aphys::add_render_func(scene, aphys::get_render_func(draw_sample_edges));

  int curr_idx = 0;
  int curr_mode = 0;  // 0: draw polygon; 1: drag points
  aphys::add_gui_mouse_input_func(gui, [&]() {
    float x = ImGui::GetMousePos().x / SCR_WIDTH;
    float y = 1.0 - ImGui::GetMousePos().y / SCR_HEIGHT;
    aphys::camera2d_screen_to_world(scene.camera, x, y);
    aphys::Vecxd p(2);
    p << x, y;

    if (curr_mode == 0) {
      if (ImGui::IsMouseClicked(0)) {
        input_shape.push_back(p);
        form_polygon(shape_ref, edges, input_shape);
        aphys::set_points_data(draw_points, shape_ref.cast<float>(),
                               aphys::MatxXf());
        if (input_shape.size() > 1) {
          aphys::set_edges_data(draw_polygon, shape_ref.cast<float>(), edges,
                                aphys::MatxXf());
        }
      }
    } else if (curr_mode == 1) {
      if (ImGui::IsMouseClicked(0)) {
        curr_idx = aphys::find_nearest_point(shape, p);
      }
      if (ImGui::IsMouseDragging(0)) {
        shape.row(curr_idx) = p.transpose();
        aphys::set_points_data(draw_points, shape.cast<float>(),
                               aphys::MatxXf());

        if (input_shape.size() > 1) {
          aphys::set_edges_data(draw_polygon, shape.cast<float>(), edges,
                                aphys::MatxXf());
        }
      }
    }
  });

  double resolution = 0.05;
  float interp = 1.0;

  aphys::add_gui_func(gui, [&]() {
    ImGui::InputDouble("Resolution", &resolution);
    ImGui::SliderFloat("Interpolation", &interp, 0.0, 1.0);
    if (ImGui::Button("Form Polygon")) {
      sample_inside_grid(shape_ref, edges, sample_points, sample_edges,
                         resolution);
      aphys::set_points_data(draw_sample_points, sample_points.cast<float>(),
                             aphys::MatxXf());
      aphys::set_edges_data(draw_sample_edges, sample_points.cast<float>(),
                            sample_edges, aphys::MatxXf());
      weights.resize(sample_points.rows(), shape_ref.rows());
      interpolated_points = sample_points;
      shape = shape_ref;
      curr_mode = 1;
    }
  });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();
    aphys::handle_gui_input(gui);

    if (curr_mode == 1) {
      WalkOnSphereSolver(sample_points, shape_ref, edges, weights, curr_idx, 16,
                         1e-6);
      nWalks++;
      weights_avged = weights / nWalks;
      interpolated_points = weights_avged * shape_ref;
      interpolated_points =
          aphys::lerp(sample_points, interpolated_points, interp);
      aphys::set_points_data(draw_sample_points,
                             interpolated_points.cast<float>(),
                             aphys::MatxXf());
      aphys::set_edges_data(draw_sample_edges,
                            interpolated_points.cast<float>(), sample_edges,
                            aphys::MatxXf());
    }

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);
    aphys::render_gui(gui);
    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
