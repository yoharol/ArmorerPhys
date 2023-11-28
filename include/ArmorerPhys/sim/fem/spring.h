#include "ArmorerPhys/type.h"

namespace aphys {

struct SpringFEM {
  static double Energy(MatxXd& verts, const MatxXd& verts_ref,
                      const Matx2i& edges, double k);
  static void Jacobian(MatxXd& verts, const MatxXd& verts_ref,
                       const Matx2i& edges, Vecxd& J, double k);
  static void Hessian(MatxXd& verts, const MatxXd& verts_ref,
                      const Matx2i& edges, MatxXd& H, double k);
};

}  // namespace aphys