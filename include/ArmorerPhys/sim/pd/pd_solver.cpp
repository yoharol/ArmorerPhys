#include "ArmorerPhys/sim/pd/pd_solver.h"

#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SVD>
#include <Eigen/IterativeLinearSolvers>

#include "ArmorerPhys/math.h"

namespace aphys {

template <>
ProjectiveDynamicsSolver<2>::ProjectiveDynamicsSolver(
    const MatxXd& verts, const MatxXd& verts_ref, const Matx3i& faces,
    const Vecxd& face_mass, const Vecxd& vert_mass,
    const MatxXd& external_force, double dt, double stiffness_hydro,
    double stiffness_devia) {
  n_verts = verts.rows();
  n_faces = faces.rows();
  L.resize(n_verts, n_verts);
  J.resize(n_verts, 2 * n_faces);
  M_h2.resize(n_verts, n_verts);
  LHS.resize(n_verts, n_verts);
  dx_ref_inv.resize(n_faces);
  P.resize(n_faces * 2, 2);

  MatxXd L_mat(n_verts, n_verts);
  L_mat.setZero();
  MatxXd J_mat(n_verts, 2 * n_faces);
  J_mat.setZero();

  for (int j = 0; j < n_faces; j++) {
    int i0 = faces(j, 0);
    int i1 = faces(j, 1);
    int i2 = faces(j, 2);

    Vecxd dx1_ref = verts_ref.row(i1) - verts_ref.row(i0);
    Vecxd dx2_ref = verts_ref.row(i2) - verts_ref.row(i0);
    MatxXd dx_ref_mat(2, 2);
    dx_ref_mat << dx1_ref, dx2_ref;
    dx_ref_mat = dx_ref_mat.inverse();
    dx_ref_inv[j] = dx_ref_mat;

    std ::vector<Tripletd> tripletListGj(4);
    tripletListGj[0] = Tripletd(i0, 0, -1.0);
    tripletListGj[1] = Tripletd(i0, 1, -1.0);
    tripletListGj[2] = Tripletd(i1, 0, 1.0);
    tripletListGj[3] = Tripletd(i2, 1, 1.0);

    SparseMatd Gj(n_verts, 2);
    Gj.setFromTriplets(tripletListGj.begin(), tripletListGj.end());
    SparseMatd dx_ref_sp = Eigen::SparseView(dx_ref_mat);
    Gj = Gj * dx_ref_sp;

    SparseMatd SjT(2, 2 * n_faces);
    SjT.insert(0, 2 * j) = 1.0;
    SjT.insert(1, 2 * j + 1) = 1.0;

    L_mat += face_mass(j) * (stiffness_hydro + stiffness_devia) * Gj *
             Gj.transpose();
    J_mat += face_mass(j) * (stiffness_hydro + stiffness_devia) * Gj * SjT;
  }
  L = L_mat.sparseView();
  J = J_mat.sparseView();
  ratio = stiffness_devia / (stiffness_hydro + stiffness_devia);

  for (int i = 0; i < n_verts; i++) {
    M_h2.insert(i, i) = vert_mass(i) / dt / dt;
  }
  M_h2.makeCompressed();

  LHS_sparse = M_h2 + L;
  sparse_solver.analyzePattern(LHS_sparse);
  sparse_solver.factorize(LHS_sparse);
}

template <>
void ProjectiveDynamicsSolver<2>::localStep(const MatxXd& verts,
                                            const Matx3i& faces) {
  MatxXd U, V;
  Vecxd S;
  MatxXd F(2, 2);
  MatxXd lhs(3, 3);
  Vecxd rhs(3);
  Vecxd delta(3);
  for (int j = 0; j < n_faces; j++) {
    int i0 = faces(j, 0);
    int i1 = faces(j, 1);
    int i2 = faces(j, 2);
    Vecxd dx1 = verts.row(i1) - verts.row(i0);
    Vecxd dx2 = verts.row(i2) - verts.row(i0);
    F << dx1, dx2;
    F = F * dx_ref_inv[j];

    Eigen::JacobiSVD<MatxXd> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();
    ssvd<2>(U, S, V);

    double sig11 = S(0);
    double sig22 = S(1);
    double d1, d2;
    d1 = 1.0 - sig11;
    d2 = 1.0 - sig22;
    double lambda = 0.0f;
    for (int iter = 0; iter < 20; iter++) {
      lhs << 1.0f, lambda, d2 + sig22,   //
          lambda, 1.0f, d1 + sig11,      //
          d2 + sig22, d1 + sig11, 0.0f;  //
      rhs << d1 + lambda * (d2 + sig22), d2 + lambda * (d1 + sig11),
          (d1 + sig11) * (d2 + sig22) - 1.0f;
      rhs = -rhs;
      delta = lhs.partialPivLu().solve(rhs);
      if ((delta(0) * delta(0) + delta(1) * delta(1)) < 1e-6) break;
      d1 += delta(0);
      d2 += delta(1);
      lambda += delta(2);
    }
    Vecxd S_new(2);
    S_new << d1 + sig11, d2 + sig22;
    MatxXd D = U * S_new.asDiagonal() * V.transpose();
    MatxXd R = U * V.transpose();
    P.block(j * 2, 0, 2, 2) = (1.0f - ratio) * D + ratio * R;
  }
}

template <>
void ProjectiveDynamicsSolver<2>::globalStep(MatxXd& verts,
                                             const MatxXd& verts_pred) {
  Eigen::MatrixXd rhs = M_h2 * verts_pred + J * P;
  Eigen::MatrixXd result = sparse_solver.solve(rhs.cast<double>());
  verts = result.cast<double>();
}

ControlledProjDynSolver::ControlledProjDynSolver(
    ProjectiveDynamicsSolver<2>* proj_solver, MatxXd control_weights) {
  pd_solver = proj_solver;
  n_controls = control_weights.rows();
  SparseMatd LHS;
  LHS = pd_solver->LHS_sparse;
  int n_verts = LHS.rows();
  assert(control_weights.cols() == n_verts);
  int n_controls = control_weights.rows();
  SparseMatd combined_LHS(n_verts + n_controls, n_verts + n_controls);
  SparseMatd control_weights_sparse = control_weights.sparseView();
  SparseMatd control_weights_sparse_T;
  control_weights_sparse_T = control_weights_sparse.transpose();
  for (int k = 0; k < LHS.outerSize(); ++k)
    for (SparseMatd::InnerIterator it(LHS, k); it; ++it)
      combined_LHS.insert(it.row(), it.col()) = it.value();
  for (int i = 0; i < control_weights.rows(); i++)
    for (int j = 0; j < control_weights.cols(); j++) {
      combined_LHS.insert(n_verts + i, j) = control_weights(i, j);
      combined_LHS.insert(j, n_verts + i) = control_weights(i, j);
    }
  sparse_solver.analyzePattern(combined_LHS);
  sparse_solver.factorize(combined_LHS);
}

void ControlledProjDynSolver::globalStep(MatxXd& verts,
                                         const MatxXd& verts_pred,
                                         const MatxXd& control_verts,
                                         MatxXd& lambda) {
  Eigen::MatrixXd rhs =
      pd_solver->M_h2 * verts_pred + pd_solver->J * pd_solver->P;
  Eigen::MatrixXd control_rhs(n_controls + pd_solver->n_verts, 2);
  control_rhs.topRows(pd_solver->n_verts) = rhs;
  control_rhs.bottomRows(n_controls) = control_verts;
  Eigen::MatrixXd result = sparse_solver.solve(control_rhs);
  verts = result.topRows(pd_solver->n_verts);
  lambda = result.bottomRows(n_controls);
}

}  // namespace aphys