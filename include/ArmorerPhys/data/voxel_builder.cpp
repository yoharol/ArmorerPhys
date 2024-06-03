#define VOXELIZER_IMPLEMENTATION
#include "ArmorerPhys/data/voxelizer.h"

#include <memory>

#include "ArmorerPhys/data/voxel_builder.h"

namespace aphys {

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
  Matx3i vert_indices_local(8, 3);
  vert_indices_local << -1, -1, -1,  //
      1, -1, -1,                     //
      -1, -1, 1,                     //
      1, -1, 1,                      //
      -1, 1, -1,                     //
      1, 1, -1,                      //
      -1, 1, 1,                      //
      1, 1, 1;

  tets.resize(point_cloud.rows() * s, 4);
  MatxXd raw_verts(point_cloud.rows() * 8, 3);
  for (int i = 0; i < point_cloud.rows(); i++) {
    for (int j = 0; j < 8; j++) {
      raw_verts.row(i * 8 + j) =
          point_cloud.row(i) + res * vert_indices_local.row(j);
    }
  }
}

}  // namespace aphys
