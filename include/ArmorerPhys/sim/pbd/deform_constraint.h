#ifndef ARMORER_SIM_PBD_CONSTRAINTS_DEFORM_H_
#define ARMORER_SIM_PBD_CONSTRAINTS_DEFORM_H_

#include "ArmorerPhys/sim/pbd/pbd_framework.h"
#include "ArmorerPhys/type.h"

#include <stdexcept>

namespace aphys {

template <int dim>
struct DeformConstraint : public PbdConstraint {
  Vecxd hydro_lambda;
  Vecxd devia_lambda;
  double hydro_tilde_alpha;
  double devia_tilde_alpha;

  DeformConstraint(MatxXd& verts, MatxXd& verts_ref, const Matx3i& faces,
                   Vecxd& face_mass, Vecxd& verts_invm, double hydro_alpha,
                   double devia_alpha, double dt);

  void preProject() override;
  void project(MatxXd& verts, MatxXd& verts_ref, const Matx3i& faces,
               Vecxd& face_mass, Vecxd& verts_invm, double dt) {
    throw std::invalid_argument(
        "DeformConstraint::project() not implemented for dim != 2 or 3");
  }
};

}  // namespace aphys

#endif  // ARMORER_SIM_PBD_CONSTRAINTS_LENGTH_H_