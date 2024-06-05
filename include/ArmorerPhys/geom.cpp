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

void get_axis_value_indices(const double value, const int dim,
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

void compute_edge_length(const MatxXd &verts, const Matx2i &edge,
                         Vecxd &length) {
  int n_edges = edge.rows();
  length.resize(n_edges);
  for (int i = 0; i < n_edges; i++) {
    int i1 = edge(i, 0);
    int i2 = edge(i, 1);
    Vecxd p1 = verts.row(i1);
    Vecxd p2 = verts.row(i2);
    length(i) = (p1 - p2).norm();
  }
}

void extract_edge(const Matx3i &faces, Matx2i &edge) {
  int n_faces = faces.rows();
  std::vector<Vec3i> raw_edges(n_faces * 3);
  for (int i = 0; i < n_faces; i++) {
    for (int k1 = 0; k1 < 3; k1++) {
      int k2 = (k1 + 1) % 3;
      int i1 = faces(i, k1);
      int i2 = faces(i, k2);
      raw_edges[i * 3 + k1] = Vec3i(std::min(i1, i2), std::max(i1, i2), i);
    }
  }
  std::sort(raw_edges.begin(), raw_edges.end(),
            [](const Vec3i &a, const Vec3i &b) {
              if (a(0) < b(0)) return true;
              if (a(0) > b(0)) return false;
              if (a(1) < b(1)) return true;
              return false;
            });

  int i = 0;
  std::vector<Vec2i> raw_edge_indices;
  auto compare = [](Vec3i e1, Vec3i e2) {
    if (e1(0) == e2(0) && e1(1) == e2(1)) return true;
    return false;
  };
  while (i < raw_edges.size()) {
    raw_edge_indices.emplace_back(Vec2i(raw_edges[i](0), raw_edges[i](1)));
    if (i + 1 < raw_edges.size() && compare(raw_edges[i], raw_edges[i + 1]))
      i += 2;
    else
      i += 1;
  }
  edge.resize(raw_edge_indices.size(), 2);
  for (int i = 0; i < raw_edge_indices.size(); i++) {
    edge.row(i) = raw_edge_indices[i];
  }
}

// check if vert is inside the tetrahedron
bool inside_tet(const Vec3d &vert, const Vec3d &tv0, const Vec3d &tv1,
                const Vec3d &tv2, const Vec3d &tv3) {
  auto same_side = [](const Vec3d &p1, const Vec3d &p2, const Vec3d &p3,
                      const Vec3d &a, const Vec3d &b) -> bool {
    Vec3d norm = (p2 - p1).cross(p3 - p1);
    double sign1 = (norm.dot(a - p1));
    double sign2 = (norm.dot(b - p1));
    return sign1 * sign2 > 0;
  };

  return same_side(tv0, tv1, tv2, tv3, vert) &&
         same_side(tv1, tv2, tv3, tv0, vert) &&
         same_side(tv2, tv3, tv0, tv1, vert) &&
         same_side(tv3, tv0, tv1, tv2, vert);
}

void compute_barycentric_tet(Vec3d &p, Vec3d &v0, Vec3d &v1, Vec3d &v2,
                             Vec3d &v3, Vec4d &bary) {
  double volume0 = compute_volume(p, v1, v2, v3);
  double volume1 = compute_volume(p, v0, v2, v3);
  double volume2 = compute_volume(p, v0, v1, v3);
  double volume3 = compute_volume(p, v0, v1, v2);
  double volume_sum = volume0 + volume1 + volume2 + volume3;
  bary << volume0 / volume_sum, volume1 / volume_sum, volume2 / volume_sum,
      volume3 / volume_sum;
}

}  // namespace aphys
