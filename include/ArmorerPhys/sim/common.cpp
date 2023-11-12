#include "ArmorerPhys/sim/common.h"

#include <vector>
#include <algorithm>
#include <iostream>

#include "ArmorerPhys/type.h"

namespace aphys {

void extract_edge(const Matx3i& faces, Matx2i& edge) {
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
            [](const Vec3i& a, const Vec3i& b) {
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

}  // namespace aphys