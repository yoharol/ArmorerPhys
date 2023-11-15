#include <cassert>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ArmorerPhys/geom.h"

namespace aphys {

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

void create_rectangle(float l, float r, int hori_count, float b, float t,
                      int vert_count, MatxXf &vertices, Matx3i &indices) {
  assert(hori_count > 0);
  assert(vert_count > 0);
  assert(l < r);
  assert(b < t);
  vertices.resize((hori_count + 1) * (vert_count + 1), 2);
  indices.resize(hori_count * vert_count * 2, 3);
  float dx = (r - l) / hori_count;
  float dy = (t - b) / vert_count;
  for (int i = 0; i < hori_count + 1; ++i) {
    for (int j = 0; j < vert_count + 1; ++j) {
      vertices.row(i * (vert_count + 1) + j) << l + i * dx, b + j * dy;
    }
  }
  for (int i = 0; i < hori_count; ++i) {
    for (int j = 0; j < vert_count; ++j) {
      int idx1 = (i * vert_count + j) * 2;
      int idx2 = idx1 + 1;
      if ((i + j) % 2 == 0) {
        indices.row(idx1) << i * (vert_count + 1) + j,
            (i + 1) * (vert_count + 1) + j, i * (vert_count + 1) + j + 1;
        indices.row(idx2) << (i + 1) * (vert_count + 1) + j,
            (i + 1) * (vert_count + 1) + j + 1, i * (vert_count + 1) + j + 1;
      } else {
        indices.row(idx1) << i * (vert_count + 1) + j,
            (i + 1) * (vert_count + 1) + j, (i + 1) * (vert_count + 1) + j + 1;
        indices.row(idx2) << i * (vert_count + 1) + j,
            (i + 1) * (vert_count + 1) + j + 1, i * (vert_count + 1) + j + 1;
      }
    }
  }
}

}  // namespace aphys
