// gl_mathplot2d.h --- ArmorerGL 2D Math Plotting Library
//

#ifndef GL_MATHPLOT2D_H
#define GL_MATHPLOT2D_H

#include <Eigen/Geometry>
#include <vector>

#include "armgl_type.h"
#include "armgl_objs.h"

namespace armgl {

Lines create_ruler2d(float x1, float y1, float x2, float y2, RGB color) {
  Lines lines = create_lines();
  Matx2f verts(2, 2);
  verts << x1, y1, x2, y2;
  set_lines_data(lines, verts, MatxXf());
  lines.mode = GL_LINES;
  lines.color = color;
  return lines;
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
  Matx2f verts(2 * (gridx + gridy), 2);
  for (int i = 0; i < gridx; i++) {
    float x = (x_max - x_min) * i / (gridx - 1) + x_min;
    verts.row(2 * i) << x, y_min;
    verts.row(2 * i + 1) << x, y_max;
  }
  for (int i = 0; i < gridy; i++) {
    float y = (y_max - y_min) * i / (gridy - 1) + y_min;
    verts.row(2 * gridx + 2 * i) << x_min, y;
    verts.row(2 * gridx + 2 * i + 1) << x_max, y;
  }
  set_lines_data(lines, verts, MatxXf());
  lines.color = color;
  lines.mode = GL_LINES;
  return lines;
}

}  // namespace armgl

#endif  // GL_MATHPLOT2D_H