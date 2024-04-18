#include "spline.h"

namespace aphys {

BezierSpline create_bezier_spline() {
  return BezierSpline{0,
                      0,
                      0,
                      MatxXf(),
                      MatxXf(),
                      MatxXf(),
                      MatxXf(),
                      Matx2i(),
                      create_points(),
                      create_lines(),
                      create_edges()};
}

void set_bezier_spline_data(int segments, BezierSpline& spline,
                            const MatxXf& control_points,
                            const MatxXf& deriv_handles, const RGB& point_color,
                            const RGB& handle_color, const RGB& spline_color) {
  spline.segments = segments;
  spline.n_controls = control_points.rows();
  int dim = control_points.cols();
  spline.control_points = control_points;
  assert(deriv_handles.rows() == spline.n_controls);
  spline.deriv_handles = deriv_handles;
  int n_p = (spline.n_controls <= 2) ? (spline.n_controls * 2)
                                     : (4 + (spline.n_controls - 2) * 3);
  spline.n_points = n_p;
  spline.render_points.resize(n_p, dim);

  update_bezier_points(spline);
  update_bezier_color(spline, point_color, handle_color, spline_color);
  update_bezier_spline(spline);
  set_points_data(spline.points, spline.render_points, spline.render_colors);
  set_edges_data(spline.edges, spline.render_points, spline.render_edges,
                 MatxXf());
}

void update_bezier_points(BezierSpline& spline) {
  int n_c = spline.n_controls;
  int dim = spline.control_points.cols();
  int n_p = spline.n_points;
  MatxXf& points = spline.render_points;
  points.row(0) = spline.control_points.row(0);
  points.row(1) = spline.control_points.row(0) + spline.deriv_handles.row(0);
  points.row(n_p - 2) =
      spline.control_points.row(n_c - 1) - spline.deriv_handles.row(n_c - 1);
  points.row(n_p - 1) = spline.control_points.row(n_c - 1);

  Matx2i& edges = spline.render_edges;
  int n_edges = (spline.n_controls <= 2) ? (spline.n_controls)
                                         : ((spline.n_controls - 2) * 2 + 2);
  edges.resize(n_edges, 2);
  edges.row(0) = Vec2i(0, 1);
  edges.row(n_edges - 1) = Vec2i(n_p - 2, n_p - 1);

  int idx = 2;
  int eids = 1;
  for (int i = 1; i < n_c - 1; i++) {
    points.row(idx++) =
        spline.control_points.row(i) - spline.deriv_handles.row(i);
    edges.row(eids++) = Vec2i(idx - 1, idx);
    points.row(idx++) = spline.control_points.row(i);
    edges.row(eids++) = Vec2i(idx - 1, idx);
    points.row(idx++) =
        spline.control_points.row(i) + spline.deriv_handles.row(i);
  }
  assert(idx == n_p - 2);
  assert(eids == n_edges - 1);
}

void update_bezier_color(BezierSpline& spline, const RGB& point_color,
                         const RGB& handle_color, const RGB& spline_color) {
  int n_p = spline.n_points;
  spline.spline.color = spline_color;
  spline.render_colors.resize(n_p, 3);
  MatxXf& color_array = spline.render_colors;
  Vecxf pc = point_color.cast<float>();
  Vecxf hc = handle_color.cast<float>();
  color_array.row(0) = pc;
  color_array.row(1) = hc;
  color_array.row(n_p - 2) = hc;
  color_array.row(n_p - 1) = pc;
  int idx = 2;
  for (int i = 1; i < spline.n_controls - 1; i++) {
    color_array.row(idx++) = hc;
    color_array.row(idx++) = pc;
    color_array.row(idx++) = hc;
  }
  assert(idx == n_p - 2);
}

template Vecxf bezier_interpolate<Vecxf>(Vecxf& P1, Vecxf& P2, Vecxf& P3,
                                         Vecxf& P4, float t);
template Vecxf bezier_interpolate_deriv<Vecxf>(Vecxf& P1, Vecxf& P2, Vecxf& P3,
                                               Vecxf& P4, float t);
template Vecxf bezier_interpolate_second_deriv<Vecxf>(Vecxf& P1, Vecxf& P2,
                                                      Vecxf& P3, Vecxf& P4,
                                                      float t);

void update_bezier_spline(BezierSpline& spline) {
  int n_seg_points = (spline.n_controls - 1) * spline.segments + 1;
  MatxXf spline_points(n_seg_points, spline.control_points.cols());
  for (int i = 0; i < spline.n_controls - 1; i++) {
    Vecxf P1 = spline.control_points.row(i);
    Vecxf P2 = spline.control_points.row(i) + spline.deriv_handles.row(i);
    Vecxf P3 =
        spline.control_points.row(i + 1) - spline.deriv_handles.row(i + 1);
    Vecxf P4 = spline.control_points.row(i + 1);
    for (int j = 0; j < spline.segments; j++) {
      float t = (float)j / (float)spline.segments;
      Vecxf point = bezier_interpolate(P1, P2, P3, P4, t);
      spline_points.row(i * spline.segments + j) = point;
    }
  }
  spline_points.row(n_seg_points - 1) =
      spline.control_points.row(spline.n_controls - 1);
  set_lines_data(spline.spline, spline_points, MatxXf());
}

RenderFunc get_render_func(BezierSpline& spline) {
  RenderFunc render_func = [&spline](Scene scene) {
    get_render_func(spline.points)(scene);
    get_render_func(spline.edges)(scene);
    get_render_func(spline.spline)(scene);
  };
  return render_func;
}

/**
 * Divide by zero equals to zero
 */
double divide(double a, double b) {
  if (b == 0)
    return 0;
  else
    return a / b;
}

Vecxd Sample_BsplineCurve(double t, const MatxXd& poly, int k) {
  // k = degree of curve + 1
  int dim = poly.cols();
  Vecxd p(dim);
  p.setZero();
  int N = poly.rows() + 1 - k;  // N=max{knot vector}
  int index = N * t + (t == 1. ? -1 : 0);
  Vecxd boor(k + 1);
  Vecxd knot(2 * k);

  const auto knot_generator = [&](int i) -> double {
    return std::clamp(i - k + 1, 0, N) / static_cast<double>(N);
  };

  // Calculate the knot vector only for adjacent 2*k values
  for (int i = 0; i < k * 2; i++) knot(i) = knot_generator(index + i);
  // Boor-Cox coefficient for k related points
  for (int i = 0; i <= k; i++) boor(i) = 0.0;
  boor(k - 1) = 1.0;

  for (int i = 2; i <= k; i++) {
    for (int j = 0; j < k; j++)
      boor(j) =
          divide(t - knot(j), knot(j + i - 1) - knot(j)) * boor(j) +
          divide(knot(j + i) - t, knot(j + i) - knot(j + 1)) * boor(j + 1);
  }
  for (int i = 0; i < k; i++) p += boor(i) * poly.row(index + i).transpose();
  return p;
}

Vecxd Bspline_Basis(double t, int n, int k, int& idx) {
  int N = n + 1 - k;  // N=max{knot vector}
  int index = N * t + (t == 1. ? -1 : 0);
  Vecxd boor(k + 1);
  Vecxd knot(2 * k);

  const auto knot_generator = [&](int i) -> double {
    return std::clamp(i - k + 1, 0, N) / static_cast<double>(N);
  };

  for (int i = 0; i < k * 2; i++) knot(i) = knot_generator(index + i);
  for (int i = 0; i <= k; i++) boor(i) = 0.0;
  boor(k - 1) = 1.0;

  for (int i = 2; i <= k; i++) {
    for (int j = 0; j < k; j++)
      boor(j) =
          divide(t - knot(j), knot(j + i - 1) - knot(j)) * boor(j) +
          divide(knot(j + i) - t, knot(j + i) - knot(j + 1)) * boor(j + 1);
  }
  idx = index;
  return boor.segment(0, k);
}

Vecxd Sample_BsplineLocalDerivative(double t, const MatxXd& poly, int k) {
  int N = poly.rows() + 1 - k;
  const auto knot_generator = [&](int i) -> double {
    return std::clamp(i - k + 1, 0, N) / static_cast<double>(N);
  };

  MatxXd Q_poly(poly.rows() - 1, poly.cols());
  int index = N * t + (t == 1. ? -1 : 0);
  for (int i = index; i < index + k - 1; i++)
    Q_poly.row(i) = (k - 1) / (knot_generator(i + k) - knot_generator(i + 1)) *
                    (poly.row(i + 1) - poly.row(i));
  int idx;
  Vecxd boor_deriv = Bspline_Basis(t, Q_poly.rows(), k - 1, idx);

  Vecxd boor(k);
  boor.setZero();
  for (int i = index; i < index + k - 1; i++) {
    boor(i + 1 - index) += (k - 1) /
                           (knot_generator(i + k) - knot_generator(i + 1)) *
                           boor_deriv(i - index);
    boor(i - index) -= (k - 1) /
                       (knot_generator(i + k) - knot_generator(i + 1)) *
                       boor_deriv(i - index);
  }

  Vecxd p(poly.cols());
  p.setZero();
  // for (int i = 0; i < k - 1; i++)
  //   p += boor_deriv(i) * Q_poly.row(index + i).transpose();
  for (int i = 0; i < k; i++) p += boor(i) * poly.row(index + i).transpose();
  return p;
  // return Sample_BsplineCurve(t, Q_poly, k - 1);
}

Vecxd Bspline_deriv_Basis(double t, int n, int k, int& idx) {
  int N = n + 1 - k;
  const auto knot_generator = [&](int i) -> double {
    return std::clamp(i - k + 1, 0, N) / static_cast<double>(N);
  };

  int index = N * t + (t == 1. ? -1 : 0);
  Vecxd boor_deriv = Bspline_Basis(t, n - 1, k - 1, idx);
  Vecxd boor(k);
  boor.setZero();
  for (int i = index; i < index + k - 1; i++) {
    boor(i + 1 - index) += (k - 1) /
                           (knot_generator(i + k) - knot_generator(i + 1)) *
                           boor_deriv(i - index);
    boor(i - index) -= (k - 1) /
                       (knot_generator(i + k) - knot_generator(i + 1)) *
                       boor_deriv(i - index);
  }
  idx = index;
  return boor;
}

Vecxd Sample_BsplineSecondDerivative(double t, const MatxXd& poly, int k) {
  int N = poly.rows() + 1 - k;
  const auto knot_generator = [&](int i) -> double {
    return std::clamp(i - k + 1, 0, N) / static_cast<double>(N);
  };

  MatxXd Q_poly(poly.rows() - 1, poly.cols());
  int index = N * t + (t == 1. ? -1 : 0);
  for (int i = index; i < index + k - 1; i++)
    Q_poly.row(i) = (k - 1) / (knot_generator(i + k) - knot_generator(i + 1)) *
                    (poly.row(i + 1) - poly.row(i));

  MatxXd Q2_poly(Q_poly.rows() - 1, Q_poly.cols());
  for (int i = index; i < index + k - 2; i++)
    Q2_poly.row(i) = (k - 2) / (knot_generator(i + k) - knot_generator(i + 2)) *
                     (Q_poly.row(i + 1) - Q_poly.row(i));
  int idx;
  Vecxd boor_second_deriv = Bspline_Basis(t, Q2_poly.rows(), k - 2, idx);
  Vecxd p(poly.cols());
  p.setZero();
  for (int i = 0; i < k - 2; i++)
    p += boor_second_deriv(i) * Q2_poly.row(index + i).transpose();

  Vecxd boor_deriv(k - 1);
  boor_deriv.setZero();
  Vecxd boor(k);
  boor.setZero();

  for (int i = index; i < index + k - 2; i++) {
    boor_deriv(i + 1 - index) +=
        (k - 2) / (knot_generator(i + k) - knot_generator(i + 2)) *
        boor_second_deriv(i - index);
    boor_deriv(i - index) -= (k - 2) /
                             (knot_generator(i + k) - knot_generator(i + 2)) *
                             boor_second_deriv(i - index);
  }
  for (int i = index; i < index + k - 1; i++) {
    boor(i + 1 - index) += (k - 1) /
                           (knot_generator(i + k) - knot_generator(i + 1)) *
                           boor_deriv(i - index);
    boor(i - index) -= (k - 1) /
                       (knot_generator(i + k) - knot_generator(i + 1)) *
                       boor_deriv(i - index);
  }

  p.setZero();
  for (int i = 0; i < k; i++) p += boor(i) * poly.row(index + i).transpose();

  return p;
  // return Sample_BsplineCurve(t, Q2_poly, k - 2);
}

Vecxd Bspline_second_deriv_Basis(double t, int n, int k, int& idx) {
  int N = n + 1 - k;
  const auto knot_generator = [&](int i) -> double {
    return std::clamp(i - k + 1, 0, N) / static_cast<double>(N);
  };
  int index = N * t + (t == 1. ? -1 : 0);

  Vecxd boor_second_deriv = Bspline_Basis(t, n - 2, k - 2, idx);
  Vecxd boor_deriv(k - 1);
  boor_deriv.setZero();
  Vecxd boor(k);
  boor.setZero();

  for (int i = index; i < index + k - 2; i++) {
    boor_deriv(i + 1 - index) +=
        (k - 2) / (knot_generator(i + k) - knot_generator(i + 2)) *
        boor_second_deriv(i - index);
    boor_deriv(i - index) -= (k - 2) /
                             (knot_generator(i + k) - knot_generator(i + 2)) *
                             boor_second_deriv(i - index);
  }
  for (int i = index; i < index + k - 1; i++) {
    boor(i + 1 - index) += (k - 1) /
                           (knot_generator(i + k) - knot_generator(i + 1)) *
                           boor_deriv(i - index);
    boor(i - index) -= (k - 1) /
                       (knot_generator(i + k) - knot_generator(i + 1)) *
                       boor_deriv(i - index);
  }
  idx = index;
  return boor;
}

}  // namespace aphys