#ifndef ARMORER_SIM_PBD_CONSTRAINTS_COMP_H_
#define ARMORER_SIM_PBD_CONSTRAINTS_COMP_H_

#include "ArmorerPhys/sim/pbd/pbd_framework.h"
#include "ArmorerPhys/type.h"

#include <stdexcept>

namespace aphys {

struct AffineConstraint2D : public PbdConstraint {
  int n_verts;
  int n_control;
  int n_constraints;
  double rotation_tilde_alpha;
  double position_tilde_alpha;
  Vecxd lambda;
  Vecxd tilde_alpha;
  Vecxd J;
  Vecxd sqrt_m;
  SparseMatd WT_M;
  SparseMatd WT_M_W;
  SparseMatd M;

  AffineConstraint2D(MatxXd& verts, MatxXd& verts_ref, MatxXd& verts_rig,
                     SparseMatd& affine_W, Vecxd& verts_mass, Vecxd& verts_invm,
                     Vecxd& Tvec, double rotation_alpha, double position_alpha,
                     double dt);
  void preProject() override;
  void project(MatxXd& verts, MatxXd& verts_ref, MatxXd& verts_rig,
               Vecxd& verts_invm, Vecxd& Tvec, double dt);
};

}  // namespace aphys

#endif  // ARMORER_SIM_PBD_CONSTRAINTS_COMP_H_