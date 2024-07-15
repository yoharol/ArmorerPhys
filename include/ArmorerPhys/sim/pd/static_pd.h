#ifndef ARMORER_SIM_STATIC_PD_H_
#define ARMORER_SIM_STATIC_PD_H_

#include <vector>

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/sim/pd/pd_solver.h"

namespace aphys {

struct ARAPInterpolate2D {
  ProjectiveDynamicsSolver2D solver;

  ARAPInterpolate2D(const MatxXd& verts, const MatxXd& verts_ref,
                    const Matx3i& faces, const Vecxd& face_mass,
                    const Vecxd& vert_mass, const MatxXd& external_force,
                    double stiffness_hydro, double stiffness_devia,
                    const Vecxi& fixed_verts);

  void local_step(const MatxXd& verts, const Matx3i& faces, MatxXd& P);

  void solver_static_shape(MatxXd& verts, const Matx3i& faces,
                           const MatxXd& fixed_pos, const MatxXd& P);
};

struct ARAPTargetShape2D {
  MatxXd S;
  MatxXd F;
  Vecxd rotate_angle;

  ARAPTargetShape2D(const MatxXd& verts, const Matx3i& faces,
                    const MatxXd& source_B);
  static void recover_deformation_map(const MatxXd& S,
                                      const Vecxd& rotate_angle, MatxXd& F);
};

}  // namespace aphys

#endif  // ARMORER_SIM_STATIC_PD_H_
