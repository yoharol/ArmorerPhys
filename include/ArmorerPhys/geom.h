#ifndef GL_GEOM_H_
#define GL_GEOM_H_

#include "ArmorerPhys/type.h"

namespace aphys {

struct Box2d {
  Matx2f bound;
  Box2d(float l, float r, float y, float b) {
    bound.resize(2, 2);
    bound << l, r, y, b;
  }
};

struct Box3d {
  Matx2f bound;
  Box3d(float l, float r, float y, float b) {
    bound.resize(2, 2);
    bound << l, r, y, b;
  }
};

MatxXf get_normals(const MatxXf &vertices, const MatxXi &indices);

void create_rectangle(float l, float r, int hori_count, float b, float t,
                      int vert_count, Matx2f &vertices, Matx3i &indices);

}  // namespace aphys

#endif  // GL_GEOM_H_