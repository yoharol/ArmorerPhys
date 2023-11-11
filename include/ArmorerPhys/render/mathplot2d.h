#ifndef GL_MATHPLOT2D_H
#define GL_MATHPLOT2D_H

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/render/objs.h"

namespace aphys {

Lines create_ruler2d(float x1, float y1, float x2, float y2, RGB color);

void set_ruler2d_data(Lines& lines, float x1, float y1, float x2, float y2);

void set_ruler2d_data(Lines& lines, Vec2f p1, Vec2f p2, float length);

Lines create_axis2d(float x_min, float x_max, float y_min, float y_max,
                    RGB color);

Lines create_grid_axis2d(float x_min, float x_max, float y_min, float y_max,
                         int gridx, int gridy, RGB color);

}  // namespace aphys

#endif  // GL_MATHPLOT2D_H