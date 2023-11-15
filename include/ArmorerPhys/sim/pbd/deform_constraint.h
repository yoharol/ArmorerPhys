#ifndef ARMORER_SIM_PBD_CONSTRAINTS_DEFORM_H_
#define ARMORER_SIM_PBD_CONSTRAINTS_DEFORM_H_

#include "ArmorerPhys/sim/pbd/pbd_framework.h"
#include "ArmorerPhys/type.h"

#include <stdexcept>

namespace aphys {

template <int dim>
struct DeformConstraint : public PbdConstraint {
  Vecxf hydro_lambda;
  Vecxf devia_lambda;
  float hydro_tilde_alpha;
  float devia_tilde_alpha;

  DeformConstraint(MatxXf& verts, MatxXf& verts_ref, const Matx3i& faces,
                   Vecxf& face_mass, Vecxf& verts_invm, float hydro_alpha,
                   float devia_alpha, float dt);

  void preProject() override;
  void project(MatxXf& verts, MatxXf& verts_ref, const Matx3i& faces,
               Vecxf& face_mass, Vecxf& verts_invm, float dt) {
    throw std::invalid_argument(
        "DeformConstraint::project() not implemented for dim != 2 or 3");
  }
};

}  // namespace aphys

#endif  // ARMORER_SIM_PBD_CONSTRAINTS_LENGTH_H_