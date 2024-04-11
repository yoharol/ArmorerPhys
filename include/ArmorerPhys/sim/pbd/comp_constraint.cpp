#include "comp_constraint.h"

namespace aphys {

AffineConstraint2D::AffineConstraint2D(MatxXd& verts, MatxXd& verts_ref,
                                       MatxXd& verts_rig, SparseMatd& affine_W,
                                       Vecxd& verts_mass, Vecxd& verts_invm,
                                       Vecxd& Tvec, double rotation_alpha,
                                       double position_alpha, double dt) {
  n_verts = verts.rows();
  n_control = affine_W.cols() / 6;
  n_constraints = affine_W.cols();
  lambda.resize(n_control * 6);
  J.resize(n_constraints);
  tilde_alpha.resize(n_constraints);
  rotation_tilde_alpha = rotation_alpha / (dt * dt);
  position_tilde_alpha = position_alpha / (dt * dt);

  M.resize(n_verts * 2, n_verts * 2);
  for (int i = 0; i < n_verts; i++) {
    M.insert(2 * i, 2 * i) = verts_mass(i);
    M.insert(2 * i + 1, 2 * i + 1) = verts_mass(i);
  }
  M.makeCompressed();

  WT_M = affine_W.transpose() * M;
  WT_M_W = WT_M * affine_W;

  for (int j = 0; j < n_constraints; j++) {
    Vecxd row = WT_M.row(j);
    if (j % 3 == 2)
      tilde_alpha(j) = position_tilde_alpha;
    else
      tilde_alpha(j) = rotation_tilde_alpha;
    J(j) = tilde_alpha(j);
    for (int i = 0; i < n_verts; i++) {
      double tmp = row(2 * i) * row(2 * i) + row(2 * i + 1) * row(2 * i + 1);
      J(j) += tmp / verts_mass(i);
    }
  }

  project_func = [&]() {
    project(verts, verts_ref, verts_rig, verts_invm, Tvec, dt);
  };
}

void AffineConstraint2D::preProject() { lambda.setZero(); }
void AffineConstraint2D::project(MatxXd& verts, MatxXd& verts_ref,
                                 MatxXd& verts_rig, Vecxd& verts_invm,
                                 Vecxd& Tvec, double dt) {
  for (int j = 0; j < n_constraints; j++) {
    Vecxd x = Vecxd::Map(verts.data(), n_verts * 2);
    SparseVecd row1 = WT_M.row(j);
    SparseVecd row2 = WT_M_W.row(j);
    double C = row1.dot(x) - row2.dot(Tvec);
    double delta_lambda = -C / J(j);
    lambda(j) += delta_lambda;
    Vecxd dx = row1 * delta_lambda;
    for (int i = 0; i < n_verts; i++) {
      verts.row(i) += verts_invm(i) * dx.segment(2 * i, 2).transpose();
    }
  }
}

}  // namespace aphys