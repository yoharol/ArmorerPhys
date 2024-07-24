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
inline T bezier_interpolate(T& P1, T& P2, T& P3, T& P4, float t) {
  return (1.0 - t) * (1.0 - t) * (1.0 - t) * P1 +
         3.0f * (1.0 - t) * (1.0 - t) * t * P2 + 3.0 * (1.0 - t) * t * t * P3 +
         t * t * t * P4;
}

inline Vec4d bezier_params(float t) {
  return Vec4d((1.0 - t) * (1.0 - t) * (1.0 - t),
               3.0 * (1.0 - t) * (1.0 - t) * t, 3.0 * (1.0 - t) * t * t,
               t * t * t);
}

template <typename T>
inline T bezier_interpolate_deriv(T& P1, T& P2, T& P3, T& P4, float t) {
  return 3.0f * (1 - t) * (1 - t) * (P2 - P1) + 6.0f * (1 - t) * t * (P3 - P2) +
         3.0f * t * t * (P4 - P3);
}

template <typename T>
inline T bezier_interpolate_second_deriv(T& P1, T& P2, T& P3, T& P4, float t) {
  return 6.0f * (1 - t) * (P3 - 2 * P2 + P1) + 6.0f * t * (P4 - 2 * P3 + P2);
}

Vecxd Sample_BsplineCurve(double t, const MatxXd& poly, int k);

Vecxd Bspline_Basis(double t, int n, int k, int& idx);

Vecxd Sample_BsplineLocalDerivative(double t, const MatxXd& poly, int k);
Vecxd Bspline_deriv_Basis(double t, int n, int k, int& idx);

Vecxd Sample_BsplineSecondDerivative(double t, const MatxXd& poly, int k);
Vecxd Bspline_second_deriv_Basis(double t, int n, int k, int& idx);

struct BezierSegment {
  aphys::Vec2d p1, p2, p3, p4;
  aphys::BezierSpline spline;

  BezierSegment(aphys::Vec2d p1, aphys::Vec2d p2, aphys::Vec2d p3,
                aphys::Vec2d p4);

  BezierSegment(aphys::MatxXd p, aphys::Vec4i id);

  void set_data(aphys::MatxXd p, aphys::Vec4i id);

  void set_bezier_data();
};

struct BezierPeices {
  std::vector<BezierSegment> peices;

  BezierPeices(aphys::MatxXd& cp, aphys::Matx4i& edges);

  void set_data(aphys::MatxXd& cp, aphys::Matx4i& edges);
};

RenderFunc get_render_func(BezierPeices& bp);

}  // namespace aphys

#endif  // GL_SPLINE_H_
