#ifndef ARMORPHYS_DATA_VOXEL_BUILDER_H_
#define ARMORPHYS_DATA_VOXEL_BUILDER_H_

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/tet.h"

namespace aphys {

// bind a surface mesh to a point cloud tet mesh
struct VoxelTet {
  int n_voxels;
  MatxXd verts;
  Matx4i tets;
  Matx3i faces;
  MatxXd pc_verts;
  Vecxi bind_index;
  MatxXd bind_weights;

  VoxelTet(const aphys::MatxXd& V, const aphys::Matx3i& faces, const double res,
           double precision = 0.0);
};

void extract_voxel_point_cloud(const aphys::MatxXd& verts,
                               const aphys::Matx3i& faces,
                               aphys::MatxXd& point_cloud, const double res,
                               double precision = 0.0);

void build_voxel_tet(const aphys::MatxXd& point_cloud, const double res,
                     aphys::MatxXd& verts, aphys::Matx4i& tets);

void bind_to_pc_tet(const aphys::MatxXd& point_cloud, const aphys::MatxXd& mesh,
                    const aphys::MatxXd& verts, const aphys::Matx4i& tets,
                    Vecxi& bind_index, MatxXd& bind_weights);

void interpolate_barycentric(aphys::MatxXd& mesh, const aphys::MatxXd& verts,
                             const aphys::Matx4i& tets, const Vecxi& bind_index,
                             const MatxXd& bind_weights);

}  // namespace aphys

#endif  // ARMORPHYS_DATA_VOXEL_BUILDER_H_
