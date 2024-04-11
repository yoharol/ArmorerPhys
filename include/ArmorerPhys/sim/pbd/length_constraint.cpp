#include "ArmorerPhys/sim/pbd/length_constraint.h"
#include "ArmorerPhys/type.h"

namespace aphys {

LengthConstraint::LengthConstraint(MatxXd& verts, const Matx2i& edges,
                                   Vecxd& rest_length, Vecxd& verts_invm,
                                   double alpha, double dt) {
  lambda.resize(edges.rows());
  tilde_alpha = alpha / (dt * dt);
  project_func = [&]() { project(verts, edges, rest_length, verts_invm, dt); };
}

void LengthConstraint::preProject() { lambda.setZero(); }

void LengthConstraint::project(MatxXd& verts, const Matx2i& edges,
                               Vecxd& rest_length, Vecxd& verts_invm,
                               double dt) {
  for (int i = 0; i < lambda.size(); i++) {
    int i1 = edges(i, 0);
    int i2 = edges(i, 1);
    Vecxd p1 = verts.row(i1);
    Vecxd p2 = verts.row(i2);
    Vecxd delta_p = p1 - p2;
    double delta_p_norm = delta_p.norm();
    double C = delta_p_norm - rest_length(i);
    Vecxd n = delta_p / delta_p_norm;

    double delta_lambda = -(C + tilde_alpha * lambda(i)) /
                          (verts_invm(i1) + verts_invm(i2) + tilde_alpha);
    lambda(i) += delta_lambda;

    verts.row(i1) += verts_invm(i1) * delta_lambda * n;
    verts.row(i2) -= verts_invm(i2) * delta_lambda * n;
  }
}

}  // namespace aphys