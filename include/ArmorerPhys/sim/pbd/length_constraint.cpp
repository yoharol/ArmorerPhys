#include "ArmorerPhys/sim/pbd/length_constraint.h"
#include "ArmorerPhys/type.h"

namespace aphys {

LengthConstraint::LengthConstraint(MatxXf& verts, const Matx2i& edges,
                                   Vecxf& rest_length, Vecxf& verts_invm,
                                   float alpha, float dt) {
  lambda.resize(edges.rows());
  tilde_alpha = alpha / (dt * dt);
  project_func = [&]() { project(verts, edges, rest_length, verts_invm, dt); };
}

void LengthConstraint::preProject() {
  for (int i = 0; i < lambda.size(); i++) {
    lambda(i) = 0.f;
  }
}

void LengthConstraint::project(MatxXf& verts, const Matx2i& edges,
                               Vecxf& rest_length, Vecxf& verts_invm,
                               float dt) {
  for (int i = 0; i < lambda.size(); i++) {
    int i1 = edges(i, 0);
    int i2 = edges(i, 1);
    Vecxf p1 = verts.row(i1);
    Vecxf p2 = verts.row(i2);
    Vecxf delta_p = p1 - p2;
    float delta_p_norm = delta_p.norm();
    float C = delta_p_norm - rest_length(i);
    Vecxf n = delta_p / delta_p_norm;

    float delta_lambda = -(C + tilde_alpha * lambda(i)) /
                         (verts_invm(i1) + verts_invm(i2) + tilde_alpha);
    lambda(i) += delta_lambda;

    verts.row(i1) += verts_invm(i1) * delta_lambda * n;
    verts.row(i2) -= verts_invm(i2) * delta_lambda * n;
  }
}

}  // namespace aphys