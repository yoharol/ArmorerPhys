#include "ArmorerPhys/tet.h"

#include <vector>
#include <algorithm>

namespace aphys {

void create_rectangular_prism(double l, double r, int x_count, double b,
                              double t, int y_count, double n, double f,
                              int z_count, MatxXd& verts, Matx4i& tet_indices) {
  constexpr int s = 6;
  verts.resize((x_count + 1) * (y_count + 1) * (z_count + 1), 3);
  tet_indices.resize(x_count * y_count * z_count * s, 4);
  double dx = (r - l) / x_count;
  double dy = (t - b) / y_count;
  double dz = (f - n) / z_count;
  auto idx = [&](int i, int j, int k) -> int {
    return i * (y_count + 1) * (z_count + 1) + j * (z_count + 1) + k;
  };
  auto tet_idx = [&](int i, int j, int k) -> int {
    return (i * y_count * z_count + j * z_count + k) * s;
  };
  for (int i = 0; i < x_count + 1; i++)
    for (int j = 0; j < y_count + 1; j++)
      for (int k = 0; k < z_count + 1; k++)
        verts.row(idx(i, j, k)) << l + i * dx, b + j * dy, n + k * dz;

  Matx4i tet_indices_local(s, 4);
  tet_indices_local << 6, 2, 5, 4,  //
      2, 0, 4, 5,                   //
      5, 1, 0, 2,                   //
      7, 5, 6, 2,                   //
      7, 5, 2, 3,                   //
      1, 2, 3, 5;
  Matx3i vert_indices_local(8, 3);
  vert_indices_local << 0, 0, 0,  //
      1, 0, 0,                    //
      0, 0, 1,                    //
      1, 0, 1,                    //
      0, 1, 0,                    //
      1, 1, 0,                    //
      0, 1, 1,                    //
      1, 1, 1;

  for (int i = 0; i < x_count; i++)
    for (int j = 0; j < y_count; j++)
      for (int k = 0; k < z_count; k++) {
        int tidx = tet_idx(i, j, k);
        for (int tet_ = 0; tet_ < s; tet_++)
          for (int vert_ = 0; vert_ < 4; vert_++) {
            int id1 = i + vert_indices_local(tet_indices_local(tet_, vert_), 0);
            int id2 = j + vert_indices_local(tet_indices_local(tet_, vert_), 1);
            int id3 = k + vert_indices_local(tet_indices_local(tet_, vert_), 2);
            tet_indices(tidx + tet_, vert_) = idx(id1, id2, id3);
          }
      }
}

void extract_surface_from_tets(const Vecxd& verts, double l, double r, double b,
                               double t, double n, double f, Matx4i& tets,
                               Matx3i& faces) {
  std::vector<Vec3i> raw_faces;
  auto on_surface = [&](Vecxd& v) -> bool {
    return (v(0) == l || v(0) == r || v(1) == b || v(1) == t || v(2) == n ||
            v(2) == f);
  };
  for (int i = 0; i < tets.rows(); i++) {
    for (int j = 0; j < 4; j++) {
      Vec3i face;
      for (int k = 0; k < 3; k++) face(k) = tets(i, (j + k) % 4);
      int surface_count = 0;
      for (int vid = 0; vid < 3; vid++) {
        Vecxd v = verts.row(face(vid));
        if (on_surface(v)) {
          surface_count++;
        }
      }
      if (surface_count == 3) raw_faces.push_back(face);
    }
  }
  faces.resize(raw_faces.size(), 3);
  for (int i = 0; i < raw_faces.size(); i++) {
    faces.row(i) = raw_faces[i].transpose();
  }
}

void extract_surface_from_tets(const int n_verts, const Matx4i& tets,
                               Matx3i& faces) {
  std::vector<Vec3i> raw_faces;
  std::vector<Vec4i> sorted_faces;
  for (int i = 0; i < tets.rows(); i++) {
    for (int j = 0; j < 4; j++) {
      Vec3i face;
      for (int k = 0; k < 3; k++) {
        face(k) = tets(i, (j + k) % 4);
      }
      raw_faces.push_back(face);
      std::sort(face.data(), face.data() + 3);
      sorted_faces.push_back(
          Vec4i(face(0), face(1), face(2), raw_faces.size() - 1));
    }
  }

  std::sort(sorted_faces.begin(), sorted_faces.end(),
            [](const Vec4i& a, const Vec4i& b) {
              if (a(0) < b(0)) return true;
              if (a(0) > b(0)) return false;
              if (a(1) < b(1)) return true;
              if (a(1) > b(1)) return false;
              return a(2) <= b(2);
            });

  std::vector<Vec3i> surface_face_indices;
  auto compare = [](Vec4i f1, Vec4i f2) {
    if (f1(0) == f2(0) && f1(1) == f2(1) && f1(2) == f2(2)) return true;
    return false;
  };

  for (int i = 0; i < sorted_faces.size(); i++) {
    if (i + 1 < sorted_faces.size() &&
        compare(sorted_faces[i], sorted_faces[i + 1])) {
      i += 1;
    } else {
      surface_face_indices.push_back(raw_faces[sorted_faces[i](3)]);
    }
  }

  faces.resize(surface_face_indices.size(), 3);
  for (int i = 0; i < surface_face_indices.size(); i++) {
    faces.row(i) = surface_face_indices[i].transpose();
  }
}

}  // namespace aphys
