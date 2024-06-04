#include "ArmorerPhys/data/voxel_builder.h"

#define VOXELIZER_IMPLEMENTATION
#include "ArmorerPhys/data/voxelizer.h"

#include <memory>
#include <iostream>
#include <vector>
#include <algorithm>

namespace aphys {

VoxelTet::VoxelTet(const aphys::MatxXd& verts, const aphys::Matx4i& tets,
                   const double res, double precision) {}

void extract_voxel_point_cloud(const aphys::MatxXd& verts,
                               const aphys::Matx3i& faces,
                               aphys::MatxXd& point_cloud, const double res,
                               double precision) {
  if (precision == 0.0) precision = res / 10.0;
  std::unique_ptr<vx_mesh> vxmesh(vx_mesh_alloc(verts.rows(), faces.size()));
  for (int i = 0; i < verts.rows(); i++) {
    vxmesh->vertices[i].x = verts(i, 0);
    vxmesh->vertices[i].y = verts(i, 1);
    vxmesh->vertices[i].z = verts(i, 2);
  }
  for (int i = 0; i < faces.rows(); i++) {
    vxmesh->indices[i * 3 + 1] = faces(i, 0);
    vxmesh->indices[i * 3 + 2] = faces(i, 1);
    vxmesh->indices[i * 3 + 3] = faces(i, 2);
  }
  std::unique_ptr<vx_point_cloud> pc_data(
      vx_voxelize_pc(vxmesh.get(), res, res, res, precision));
  point_cloud.resize(pc_data->nvertices, 3);
  for (int i = 0; i < pc_data->nvertices; i++) {
    point_cloud.row(i) = aphys::RowVec3d(
        pc_data->vertices[i].x, pc_data->vertices[i].y, pc_data->vertices[i].z);
  }
}

void build_voxel_tet(const aphys::MatxXd& point_cloud, const double res,
                     aphys::MatxXd& verts, aphys::Matx4i& tets) {
  constexpr int s = 6;
  Matx4i tet_indices_local(s, 4);
  tet_indices_local << 6, 2, 5, 4,  //
      2, 0, 4, 5,                   //
      5, 1, 0, 2,                   //
      7, 5, 6, 2,                   //
      7, 5, 2, 3,                   //
      1, 2, 3, 5;
  MatxXd vert_indices_local(8, 3);
  vert_indices_local << -1, -1, -1,  //
      1, -1, -1,                     //
      -1, -1, 1,                     //
      1, -1, 1,                      //
      -1, 1, -1,                     //
      1, 1, -1,                      //
      -1, 1, 1,                      //
      1, 1, 1;

  tets.resize(point_cloud.rows() * s, 4);
  std::vector<Vec4d> raw_verts(point_cloud.rows() * 8);
  for (int i = 0; i < point_cloud.rows(); i++) {
    for (int j = 0; j < 8; j++) {
      raw_verts[i * 8 + j].head(3) =
          (point_cloud.row(i) + 0.5 * res * vert_indices_local.row(j))
              .transpose();
      raw_verts[i * 8 + j](3) = i * 8 + j;
    }
    for (int j = 0; j < s; j++) {
      for (int k = 0; k < 4; k++) {
        tets(i * s + j, k) = i * 8 + tet_indices_local(j, k);
      }
    }
  }

  // sort raw_verts by x, y, z
  auto approx = [](const double a, const double b) -> bool {
    return std::abs(a - b) < 1e-5;
  };
  auto order = [&approx](const Vec4d& a, const Vec4d& b) {
    if (!approx(a(2), b(2))) return a(2) < b(2);
    if (!approx(a(1), b(1))) return a(1) < b(1);
    return a(0) < b(0);
  };

  std::sort(raw_verts.begin(), raw_verts.end(), order);

  auto compare = [](const Vec4d& a, const Vec4d& b) {
    if (a.head(3).isApprox(b.head(3), 1e-5)) return true;
    return false;
  };
  std::vector<int> corres_idx(point_cloud.rows() * 8);
  std::vector<Vec3d> sorted_verts_raw;

  int idx = 0;
  while (idx < raw_verts.size()) {
    sorted_verts_raw.emplace_back(raw_verts[idx].head(3));
    corres_idx[raw_verts[idx](3)] = sorted_verts_raw.size() - 1;
    idx++;
    while (idx < raw_verts.size() &&
           compare(raw_verts[idx - 1], raw_verts[idx])) {
      corres_idx[raw_verts[idx](3)] = sorted_verts_raw.size() - 1;
      idx++;
    }
  }

  for (int i = 0; i < tets.rows(); i++) {
    for (int j = 0; j < 4; j++) {
      tets(i, j) = corres_idx[tets(i, j)];
    }
  }

  verts.resize(sorted_verts_raw.size(), 3);
  for (int i = 0; i < sorted_verts_raw.size(); i++) {
    verts.row(i) = sorted_verts_raw[i].transpose();
  }
}

}  // namespace aphys
