#ifndef GL_GEOM_H_
#define GL_GEOM_H_

#include <cassert>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "type.h"

namespace asim {

MatxXf get_normals(const MatxXf &vertices, const MatxXi &indices) {
  assert(indices.cols() == 3);
  assert(indices.rows() > 0);
  assert(vertices.cols() == 3);
  assert(vertices.rows() > 0);

  MatxXf normals = MatxXf::Zero(vertices.rows(), 3);
  for (int i = 0; i < indices.rows(); ++i) {
    Vec3f v0 = vertices.row(indices(i, 0));
    Vec3f v1 = vertices.row(indices(i, 1));
    Vec3f v2 = vertices.row(indices(i, 2));
    Vec3f normal = (v1 - v0).cross(v2 - v0);
    normals.row(indices(i, 0)) += normal;
    normals.row(indices(i, 1)) += normal;
    normals.row(indices(i, 2)) += normal;
  }
  for (int i = 0; i < normals.rows(); ++i) {
    normals.row(i).normalize();
  }
  return normals;
}

}  // namespace asim

#endif  // GL_GEOM_H_