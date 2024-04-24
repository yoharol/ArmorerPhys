#include <cassert>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ArmorerPhys/geom.h"

namespace aphys {

int find_nearest_point(const MatxXd &points, const Vecxd &point) {
  assert(points.cols() == point.size());
  int n_points = points.rows();
  double min_dist = (points.row(0).transpose() - point).norm();
  int min_idx = 0;
  for (int i = 1; i < n_points; ++i) {
    double dist = (points.row(i).transpose() - point).norm();
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

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

void create_rectangle(double l, double r, int hori_count, double b, double t,
                      int vert_count, MatxXd &vertices, Matx3i &indices) {
  assert(hori_count > 0);
  assert(vert_count > 0);
  assert(l < r);
  assert(b < t);
  vertices.resize((hori_count + 1) * (vert_count + 1), 2);
  indices.resize(hori_count * vert_count * 2, 3);
  double dx = (r - l) / hori_count;
  double dy = (t - b) / vert_count;
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

void get_rectangle_axis_indices(const double value, const int dim,
                                const MatxXd &vertices, Vecxi &axis_indices) {
  std::vector<int> indices;
  for (int i = 0; i < vertices.rows(); i++) {
    if (vertices(i, dim) == value) {
      indices.push_back(i);
    }
  }
  axis_indices = Vecxi::Map(indices.data(), indices.size());
}

AffineControls create_affine_controls(MatxXd &points_ref_position) {
  int n_control_points = points_ref_position.rows();
  int dim = points_ref_position.cols();
  return AffineControls{n_control_points, dim, points_ref_position,
                        MatxXd(n_control_points * dim, dim + 1),
                        Vecxd(n_control_points * dim * (dim + 1))};
}

void affine_organize_Tvec(AffineControls &controls) {
  controls.Tvec = Vecxd::Map(controls.T.data(), controls.T.size());
}

void update_affine_rotation(AffineControls &controls, int idx, double angle) {
  assert(controls.dim == 2);
  Eigen::AngleAxisd rot(angle, Eigen::Vector3d::UnitZ());
  Eigen::Matrix2d R = rot.toRotationMatrix().block(0, 0, 2, 2);
  controls.T.block(idx * 2, 0, 2, 2) = R;
}

void update_affine_rotation(AffineControls &controls, Vecxd &angles) {
  for (int i = 0; i < controls.n_control_points; i++) {
    update_affine_rotation(controls, i, angles(i));
  }
}

void update_affine_translation(AffineControls &controls, int idx,
                               Vecxd &translation) {
  int dim = controls.dim;
  controls.T.block(idx * dim, dim, dim, 1) = translation;
}

void update_affine_from_world_translation(AffineControls &controls, int idx,
                                          Vecxd &translation) {
  int dim = controls.dim;
  MatxXd rot_i = controls.T.block(idx * dim, 0, dim, dim);
  Vecxd ref_pos = controls.points_ref_position.row(idx).transpose();
  controls.T.block(idx * dim, dim, dim, 1) = (translation - rot_i * ref_pos);
}

void update_affine_from_world_translation(AffineControls &controls,
                                          MatxXd &translation) {
  for (int i = 0; i < controls.n_control_points; i++) {
    Vecxd trans = translation.row(i).transpose();
    update_affine_from_world_translation(controls, i, trans);
  }
}

}  // namespace aphys
