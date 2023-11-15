#include "ArmorerPhys/type.h"

namespace aphys {

struct SpringFEM {
  static float Energy(MatxXf& verts, const MatxXf& verts_ref,
                      const Matx2i& edges, float k);
  static void Jacobian(MatxXf& verts, const MatxXf& verts_ref,
                       const Matx2i& edges, Vecxf& J, float k);
  static void Hessian(MatxXf& verts, const MatxXf& verts_ref,
                      const Matx2i& edges, MatxXf& H, float k);
};

}  // namespace aphys