
#ifndef ARMORER_USD_MESH_H
#define ARMORER_USD_MESH_H

#include <string>
#include <Eigen/Core>

#include "pxr/usd/usdGeom/mesh.h"

#include "ArmorerPhys/render/objs.h"

namespace aphys {

struct UsdMesh {
  std::string primPath;
  pxr::UsdGeomMesh geom;
};

UsdMesh create_mesh(const std::string& primPath,
                    const pxr::UsdStageRefPtr& stage) {
  UsdMesh mesh;
  mesh.primPath = primPath;
  mesh.geom = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath(primPath));
  return mesh;
}

void set_mesh_data(UsdMesh& mesh, const Eigen::MatrixX3f& positions,
                   const Eigen::MatrixX3i& faces) {
  mesh.geom.GetPointsAttr().Set(VtMat3Xf(positions));
  mesh.geom.GetFaceVertexCountsAttr().Set(VtVecX(faces.rows(), 3));
  mesh.geom.GetFaceVertexIndicesAttr().Set(VtVecXFlatten<int>(faces));
}

}  // namespace aphys

#endif  // ARMORER_USD_MESH_H
