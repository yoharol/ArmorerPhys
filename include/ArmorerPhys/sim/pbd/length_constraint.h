#ifndef ARMORER_SIM_PBD_CONSTRAINTS_LENGTH_H_
#define ARMORER_SIM_PBD_CONSTRAINTS_LENGTH_H_

#include "ArmorerPhys/sim/pbd/pbd_framework.h"
#include "ArmorerPhys/type.h"

namespace aphys {

struct LengthConstraint : public PbdConstraint {
  Vecxd lambda;
  double tilde_alpha;

  LengthConstraint(MatxXd& verts, const Matx2i& edges, Vecxd& rest_length,
                   Vecxd& verts_invm, double alpha, double dt);

  void preProject() override;
  void project(MatxXd& verts, const Matx2i& edges, Vecxd& rest_length,
               Vecxd& verts_invm, double dt);
};

}  // namespace aphys

#endif  // ARMORER_SIM_PBD_CONSTRAINTS_LENGTH_H_