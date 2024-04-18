#ifndef GL_SPLINE_H_
#define GL_SPLINE_H_

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/render/buffer.h"
#include "ArmorerPhys/render/shader.h"
#include "ArmorerPhys/render/scene.h"
#include "ArmorerPhys/render/objs.h"

namespace aphys {

struct BezierSpline {
  int n_controls;
  int n_points;
  int segments;
  MatxXf control_points;
  MatxXf deriv_handles;
  MatxXf render_points;
  MatxXf render_colors;
  Matx2i render_edges;
  Points points;
  Lines spline;
  Edges edges;
};

BezierSpline create_bezier_spline();

void set_bezier_spline_data(int segments, BezierSpline& spline,
                            const MatxXf& control_points,
                            const MatxXf& deriv_handles, const RGB& point_color,
                            const RGB& handle_color, const RGB& spline_color);
void update_bezier_points(BezierSpline& spline);
void update_bezier_color(BezierSpline& spline, const RGB& point_color,
                         const RGB& handle_color, const RGB& spline_color);
void update_bezier_spline(BezierSpline& spline);

RenderFunc get_render_func(BezierSpline& spline);

template <typename T>
T bezier_interpolate(T& P1, T& P2, T& P3, T& P4, float t) {
  return (1 - t) * (1 - t) * (1 - t) * P1 + 3.0f * (1 - t) * (1 - t) * t * P2 +
         3 * (1 - t) * t * t * P3 + t * t * t * P4;
}

template <typename T>
T bezier_interpolate_deriv(T& P1, T& P2, T& P3, T& P4, float t) {
  return 3.0f * (1 - t) * (1 - t) * (P2 - P1) + 6.0f * (1 - t) * t * (P3 - P2) +
         3.0f * t * t * (P4 - P3);
}

template <typename T>
T bezier_interpolate_second_deriv(T& P1, T& P2, T& P3, T& P4, float t) {
  return 6.0f * (1 - t) * (P3 - 2 * P2 + P1) + 6.0f * t * (P4 - 2 * P3 + P2);
}

Vecxd Sample_BsplineCurve(double t, const MatxXd& poly, int k);

Vecxd Bspline_Basis(double t, int n, int k, int& idx);

Vecxd Sample_BsplineLocalDerivative(double t, const MatxXd& poly, int k);
Vecxd Bspline_deriv_Basis(double t, int n, int k, int& idx);

Vecxd Sample_BsplineSecondDerivative(double t, const MatxXd& poly, int k);
Vecxd Bspline_second_deriv_Basis(double t, int n, int k, int& idx);

}  // namespace aphys

#endif  // GL_SPLINE_H_
