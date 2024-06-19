#ifndef GL_MATHPLOT2D_H
#define GL_MATHPLOT2D_H

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/render/objs.h"

#include "ArmorerPhys/geom.h"

namespace aphys {

Lines create_ruler2d(float x1, float y1, float x2, float y2, RGB color);

void set_ruler2d_data(Lines &lines, float x1, float y1, float x2, float y2);

void set_ruler2d_data(Lines &lines, Vec2f p1, Vec2f p2, float length);

Lines create_axis2d(float x_min, float x_max, float y_min, float y_max,
                    RGB color);

Lines create_grid_axis2d(float x_min, float x_max, float y_min, float y_max,
                         int gridx, int gridy, RGB color);

Lines create_axis3d(float x_min, float x_max, float y_min, float y_max,
                    float z_min, float z_max, RGB color);

struct ControlPoint {
  int n_control_points;
  float scale;
  Points points;
  Edges directions;
};

ControlPoint create_control_point();
void set_control_point_data(ControlPoint &control_points,
                            AffineControls &control,
                            const MatxXf &per_point_color);
RenderFunc get_render_func(ControlPoint &control_points);

}  // namespace aphys

#endif  // GL_MATHPLOT2D_H
