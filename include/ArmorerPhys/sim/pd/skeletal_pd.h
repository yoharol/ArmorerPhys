#ifndef ARMORER_SIM_SKELETAL_PD_H_
#define ARMORER_SIM_SKELETAL_PD_H_

#include <vector>

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/sim/pd/pd_solver.h"

namespace aphys {

// Support:
//  1. Only affine reduction (partial or fully)
//  2. Nonlinear constraints (rigid constraints)
struct SkeletalProjectiveDynamics3D : ProjectiveDynamicsSolver3D {
  SkeletalProjectiveDynamics3D(const MatxXd& verts, const MatxXd& verts_ref,
                               const Matx4i& tets, const Vecxd& tet_mass,
                               const Vecxd& vert_mass,
                               const MatxXd& external_force, double dt,
                               double stiffness_hydro, double stiffness_devia,
                               const SparseMatd& weight_mat);

  void globalStep(const SparseMatd& weight_mat, MatxXd& verts,
                  const MatxXd& verts_pred);
};

struct SkeletalProjectiveDynamics2D : ProjectiveDynamicsSolver2D {
  SkeletalProjectiveDynamics2D(const MatxXd& verts, const MatxXd& verts_ref,
                               const Matx3i& faces, const Vecxd& face_mass,
                               const Vecxd& vert_mass,
                               const MatxXd& external_force, double dt,
                               double stiffness_hydro, double stffness_devia,
                               const SparseMatd& weight_mat);

  void globalStep(const SparseMatd& weight_mat, MatxXd& verts,
                  const MatxXd& verts_pred);
};

}  // namespace aphys

#endif  // ARMORER_SIM_SKELETAL_PD_H_
