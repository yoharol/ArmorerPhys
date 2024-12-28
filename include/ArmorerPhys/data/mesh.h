#ifndef ARMORPHYS_DATA_MESH_H_
#define ARMORPHYS_DATA_MESH_H_

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/glmath.h"

namespace aphys {

struct ClothMesh {
  int n_verts;
  int n_faces;
  int n_edges;
  MatxXd verts;
  Matx3i faces;
  Matx4i edges;
  Vecxd face_mass;
  Vecxd verts_mass;
  Matx2i edge_faces;  // store the two faces that share the edge

  void Initialize();
};

}  // namespace aphys

#endif  // ARMORPHYS_DATA_MESH_H_
