#ifndef ARMORPHYS_DATA_VOXEL_BUILDER_H_
#define ARMORPHYS_DATA_VOXEL_BUILDER_H_

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/tet.h"

namespace aphys {

struct VoxelTet {
  int n_voxels;
  TetMesh tet_mesh;
  MatxXd voxel_verts;

  VoxelTet(const aphys::MatxXd& verts, const aphys::Matx4i& tets,
           const double res, double precision = 0.0);
};

void extract_voxel_point_cloud(const aphys::MatxXd& verts,
                               const aphys::Matx3i& faces,
                               aphys::MatxXd& point_cloud, const double res,
                               double precision = 0.0);

void build_voxel_tet(const aphys::MatxXd& point_cloud, const double res,
                     aphys::MatxXd& verts, aphys::Matx4i& tets);

}  // namespace aphys

#endif  // ARMORPHYS_DATA_VOXEL_BUILDER_H_
