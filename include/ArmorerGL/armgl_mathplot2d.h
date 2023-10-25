// gl_mathplot2d.h --- ArmorerGL 2D Math Plotting Library
//

#ifndef GL_MATHPLOT2D_H
#define GL_MATHPLOT2D_H

#include <Eigen/Geometry>
#include <vector>

#include "armgl_type.h"
#include "armgl_objs.h"

namespace agl {

Lines create_ruler2d(float x1, float y1, float x2, float y2, RGB color) {
  Lines lines = create_lines();
  Matx2f verts(2, 2);
  verts << x1, y1, x2, y2;
  set_lines_data(lines, verts, MatxXf());
  lines.mode = GL_LINES;
  lines.color = color;
  return lines;
}

void set_ruler2d_data(Lines& lines, float x1, float y1, float x2, float y2) {
  Matx2f verts(2, 2);
  verts << x1, y1, x2, y2;
  set_lines_data(lines, verts, MatxXf());
}

void set_ruler2d_data(Lines& lines, Vec2f p1, Vec2f p2, float length) {
  Vec2f v = p2 - p1;
  v.normalize();
  Vec2f v1 = p1 + v * length;
  Vec2f v2 = p1 + v * (-length);
  set_ruler2d_data(lines, v1(0), v1(1), v2(0), v2(1));
}

Lines create_axis2d(float x_min, float x_max, float y_min, float y_max,
                    RGB color) {
  Lines lines = create_lines();
  Matx2f verts(4, 2);
  verts << x_min, 0.f,  //
      x_max, 0.f,       //
      0.f, y_min,       //
      0.f, y_max;       //
  set_lines_data(lines, verts, MatxXf());
  lines.color = color;
  lines.mode = GL_LINES;
  return lines;
}

Lines create_grid_axis2d(float x_min, float x_max, float y_min, float y_max,
                         int gridx, int gridy, RGB color) {
  Lines lines = create_lines();
  Matx2f verts(2 * (gridx + gridy + 2), 2);
  for (int i = 0; i <= gridx; i++) {
    float x = (x_max - x_min) * i / gridx + x_min;
    verts.row(2 * i) << x, y_min;
    verts.row(2 * i + 1) << x, y_max;
  }
  for (int i = 0; i <= gridy; i++) {
    float y = (y_max - y_min) * i / gridy + y_min;
    verts.row(2 * (gridx + 1) + 2 * i) << x_min, y;
    verts.row(2 * (gridx + 1) + 2 * i + 1) << x_max, y;
  }
  set_lines_data(lines, verts, MatxXf());
  lines.color = color;
  lines.mode = GL_LINES;
  return lines;
}

}  // namespace agl

#endif  // GL_MATHPLOT2D_H