#include "ArmorerPhys/data/mesh.h"

#include "ArmorerPhys/geom.h"
#include "ArmorerPhys/SimCore.h"

#include <algorithm>

namespace aphys {

void ClothMesh::Initialize() {
  n_verts = verts.rows();
  n_faces = faces.rows();

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
  std::vector<Vec2i> raw_edge_faces;
  auto compare = [](Vec3i e1, Vec3i e2) {
    if (e1(0) == e2(0) && e1(1) == e2(1)) return true;
    return false;
  };
  auto find_extra_vert = [&](int face_idx, int i1, int i2) -> int {
    for (int i = 0; i < 3; i++) {
      int idx = faces(face_idx, i);
      if (idx != i1 && idx != i2) {
        return idx;
      }
    }
    return -1;
  };
  while (i < raw_edges.size()) {
    raw_edge_indices.emplace_back(Vec2i(raw_edges[i](0), raw_edges[i](1)));

    if (i + 1 < raw_edges.size() && compare(raw_edges[i], raw_edges[i + 1])) {
      raw_edge_faces.emplace_back(Vec2i(raw_edges[i](2), raw_edges[i + 1](2)));
      i += 2;
    } else {
      raw_edge_faces.emplace_back(Vec2i(raw_edges[i](2), -1));
      i += 1;
    }
  }
  edges.resize(raw_edge_indices.size(), 4);
  edges.fill(-1);
  for (int i = 0; i < raw_edge_indices.size(); i++) {
    edges(i, 0) = raw_edge_indices[i](0);
    edges(i, 1) = raw_edge_indices[i](1);
    if (raw_edge_faces[i](0) != -1)
      edges(i, 2) =
          find_extra_vert(raw_edge_faces[i](0), edges(i, 0), edges(i, 1));
    if (raw_edge_faces[i](1) != -1)
      edges(i, 3) =
          find_extra_vert(raw_edge_faces[i](1), edges(i, 0), edges(i, 1));
  }
  edge_faces.resize(raw_edge_faces.size(), 2);
  for (int i = 0; i < raw_edge_faces.size(); i++) {
    edge_faces.row(i) = raw_edge_faces[i];
  }

  n_edges = edges.rows();
  compute_mesh_mass(verts, faces, face_mass, verts_mass);
}

}  // namespace aphys
