#ifndef ARMORER_USD_POINTS_H
#define ARMORER_USD_POINTS_H

#include <string>
#include <Eigen/Core>

#include "pxr/usd/usdGeom/points.h"

#include "types.h"

namespace asim {

struct Points {
  std::string primPath;
  pxr::UsdGeomPoints geom;
  float widths;
};

Points create_points(const std::string& primPath,
                     const pxr::UsdStageRefPtr& stage, float widths) {
  Points points;
  points.primPath = primPath;
  points.geom = pxr::UsdGeomPoints::Define(stage, pxr::SdfPath(primPath));
  points.widths = widths;
  points.geom.GetWidthsAttr().Set(pxr::VtFloatArray{widths});
  points.geom.SetWidthsInterpolation(pxr::UsdGeomTokens->constant);
  return points;
}

void set_points_data(Points& points, const Eigen::MatrixX3f& positions) {
  points.geom.GetPointsAttr().Set(VtMat3Xf(positions));
}

void read_position_data(Points& points, Eigen::MatrixX3f& position) {
  pxr::VtVec3fArray vt_positions;
  points.geom.GetPointsAttr().Get(&vt_positions);
  ReadVtMat3Xf(vt_positions, position);
}

}  // namespace asim

#endif  // ARMORER_USD_POINTS_H