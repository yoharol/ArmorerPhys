#ifndef GL_GEOM_H_
#define GL_GEOM_H_

#include "ArmorerPhys/type.h"

namespace aphys {

struct Box2d {
  Matx2d bound;
  Box2d(double l, double r, double y, double b) {
    bound.resize(2, 2);
    bound << l, r, y, b;
  }
};

struct Box3d {
  Matx2d bound;
  Box3d(double l, double r, double y, double b) {
    bound.resize(2, 2);
    bound << l, r, y, b;
  }
};

MatxXf get_normals(const MatxXf &vertices, const MatxXi &indices);

void create_rectangle(double l, double r, int hori_count, double b, double t,
                      int vert_count, MatxXd &vertices, Matx3i &indices);

}  // namespace aphys

#endif  // GL_GEOM_H_