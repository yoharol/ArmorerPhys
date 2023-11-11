#ifndef ARMORER_USD_STAGE_H
#define ARMORER_USD_STAGE_H

#include <string>

#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/reference.h"

namespace aphys {

struct Stage {
  std::string path;
  std::string rootPrimPath;
  pxr::UsdStageRefPtr stage;
};

Stage create_stage(const std::string& path, const std::string& rootPrimPath) {
  Stage stage;
  stage.path = path;
  stage.rootPrimPath = rootPrimPath;
  stage.stage = pxr::UsdStage::CreateNew(stage.path);
  pxr::UsdGeomSetStageUpAxis(stage.stage, pxr::UsdGeomTokens->y);
  return stage;
}

Stage read_stage(const std::string& path) {
  Stage stage;
  stage.path = path;
  stage.stage = pxr::UsdStage::Open(stage.path);
  return stage;
}

void save_stage(Stage& stage) { stage.stage->Save(); }

void set_stage_timeline(Stage& stage, float start, float end, float fps) {
  stage.stage->SetStartTimeCode(start);
  stage.stage->SetEndTimeCode(end);
  stage.stage->SetFramesPerSecond(fps);
}

std::string get_stage_usda(Stage& stage) {
  std::string usda;
  stage.stage->ExportToString(&usda);
  return usda;
}

}  // namespace aphys

#endif  // ARMORER_USD_STAGE_H