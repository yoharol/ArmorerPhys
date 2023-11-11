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

}  // namespace aphys

#endif  // GL_GEOM_H_