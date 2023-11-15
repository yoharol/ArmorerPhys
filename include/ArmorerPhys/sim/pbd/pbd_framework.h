#ifndef ARMORER_SIM_PBD_FRAMEWORK_H_
#define ARMORER_SIM_PBD_FRAMEWORK_H_

#include <vector>
#include <memory>
#include <functional>

#include "ArmorerPhys/type.h"

namespace aphys {

typedef std::function<void()> ConstraintProjectFunc;

struct PbdConstraint {
  virtual void preProject() = 0;
  virtual ~PbdConstraint() {}
  ConstraintProjectFunc project_func;
};

struct PbdFramework {
  std::vector<std::shared_ptr<PbdConstraint>> constraints;
  std::vector<ConstraintProjectFunc> project_funcs;

  void addConstraint(std::shared_ptr<PbdConstraint> constraint) {
    constraints.push_back(constraint);
    project_funcs.push_back(constraint->project_func);
  }

  void preProjectConstraints() {
    for (auto& constraint : constraints) {
      constraint->preProject();
    }
  }

  void projectConstraints() {
    for (auto& constraint : constraints) {
      constraint->project_func();
    }
  }

  void pbdPredict(MatxXf& pos, MatxXf& vel, MatxXf& pos_cache, Vecxf& vert_invm,
                  Vecxf& external_force, float dt);

  void pbdUpdateVelocity(MatxXf& pos, MatxXf& vel, MatxXf& pos_cache, float dt,
                         float damping = 0.0f);
};

}  // namespace aphys

#endif  // ARMORER_SIM_PBD_FRAMEWORK_H_
