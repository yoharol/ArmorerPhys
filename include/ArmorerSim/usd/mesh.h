
#ifndef ARMORER_USD_MESH_H
#define ARMORER_USD_MESH_H

#include <string>
#include <Eigen/Core>

#include "pxr/usd/usdGeom/mesh.h"

#include "ArmorerGL/armgl_objs.h"

namespace asim {

struct Mesh {
  std::string primPath;
  pxr::UsdGeomMesh geom;
};

Mesh create_mesh(const std::string& primPath,
                 const pxr::UsdStageRefPtr& stage) {
  Mesh mesh;
  mesh.primPath = primPath;
  mesh.geom = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath(primPath));
  return mesh;
}

void set_mesh_data(Mesh& mesh, const Eigen::MatrixX3f& positions,
                   const Eigen::MatrixX3i& faces) {
  mesh.geom.GetPointsAttr().Set(VtMat3Xf(positions));
  mesh.geom.GetFaceVertexCountsAttr().Set(VtVecX(faces.rows(), 3));
  mesh.geom.GetFaceVertexIndicesAttr().Set(VtVecXFlatten<int>(faces));
}

}  // namespace asim

#endif  // ARMORER_USD_MESH_H