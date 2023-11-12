#ifndef ARMORER_SIM_PBD_CONSTRAINTS_LENGTH_H_
#define ARMORER_SIM_PBD_CONSTRAINTS_LENGTH_H_

#include "ArmorerPhys/sim/pbd/pbd_framework.h"
#include "ArmorerPhys/type.h"

namespace aphys {
struct LengthConstraint : public PbdConstraint {
  int N;
  Vecxf lambda;
  float tilde_alpha;

  LengthConstraint(int N, MatxXf& verts, const Vecxi& edges, Vecxf& rest_length,
                   Vecxf& verts_invm, float alpha, float dt);

  void preProject() override;
  void project(MatxXf& verts, const Vecxi& edges, Vecxf& rest_length,
               Vecxf& verts_invm, float dt);
};
}  // namespace aphys

#endif  // ARMORER_SIM_PBD_CONSTRAINTS_LENGTH_H_