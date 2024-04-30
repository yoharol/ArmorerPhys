#ifndef ARMORER_SIM_FEM_NEOHOOKEAN_H_
#define ARMORER_SIM_FEM_NEOHOOKEAN_H_

#include "ArmorerPhys/type.h"

namespace aphys {

void diff_d_F_d_x(const Mat2d& B, Eigen::Matrix<double, 4, 6>& dFdx);
void diff_d_F_d_x(const Mat3d& B, MatxXd& dFdx);

// stable elasticity model
// https://graphics.pixar.com/library/StableElasticity/paper.pdf
struct NeoHookeanFEM2D {
  static void project_B(const MatxXd& verts, const MatxXd& verts_ref,
                        const Matx3i& faces, MatxXd& B);
  static void project_F(const MatxXd& verts, const Matx3i& faces,
                        const MatxXd& B, MatxXd& F);
  static double Energy(const MatxXd& F, const Vecxd& face_volume, double mu,
                       double lambda);
  static void Jacobian(const MatxXd& F, const MatxXd& B, const Matx3i& faces,
                       const Vecxd& face_volume, Vecxd& J, double mu,
                       double lambda);
  static void Hessian(const MatxXd& F, const MatxXd& B, const Matx3i& faces,
                      const Vecxd& face_volume, MatxXd& H, double mu,
                      double lambda);
};

}  // namespace aphys

#endif  // ARMORER_SIM_FEM_NEOHOOKEAN_H_