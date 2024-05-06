#ifndef ARMORER_SIM_FEM_SPRING_H_
#define ARMORER_SIM_FEM_SPRING_H_

#include "ArmorerPhys/type.h"

namespace aphys {

struct SpringFEM {
  static double Energy(MatxXd& verts, const MatxXd& verts_ref,
                       const Matx2i& edges, double k);
  static void Jacobian(const MatxXd& verts, const MatxXd& verts_ref,
                       const Matx2i& edges, Vecxd& J, double k);
  static void Hessian(const MatxXd& verts, const MatxXd& verts_ref,
                      const Matx2i& edges, MatxXd& H, double k);
};

}  // namespace aphys

#endif  // ARMORER_SIM_FEM_SPRING_H_