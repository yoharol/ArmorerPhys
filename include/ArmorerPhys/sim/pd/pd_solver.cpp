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
    const MatxXf& verts, const MatxXf& verts_ref, const Matx3i& faces,
    const Vecxf& face_mass, const Vecxf& vert_mass,
    const MatxXf& external_force, float dt, float stiffness_hydro,
    float stiffness_devia) {
  n_verts = verts.rows();
  n_faces = faces.rows();
  L.resize(n_verts, n_verts);
  J.resize(n_verts, 2 * n_faces);
  M_h2.resize(n_verts, n_verts);
  LHS.resize(n_verts, n_verts);
  dx_ref_inv.resize(n_faces);
  P.resize(n_faces * 2, 2);

  for (int j = 0; j < n_faces; j++) {
    int i0 = faces(j, 0);
    int i1 = faces(j, 1);
    int i2 = faces(j, 2);

    Vecxf dx1_ref = verts_ref.row(i1) - verts_ref.row(i0);
    Vecxf dx2_ref = verts_ref.row(i2) - verts_ref.row(i0);
    MatxXf dx_ref_mat(2, 2);
    dx_ref_mat << dx1_ref, dx2_ref;
    dx_ref_mat = dx_ref_mat.inverse();
    dx_ref_inv[j] = dx_ref_mat;

    std ::vector<Tripletf> tripletListGj(4);
    tripletListGj[0] = Tripletf(i0, 0, -1.0f);
    tripletListGj[1] = Tripletf(i0, 1, -1.0f);
    tripletListGj[2] = Tripletf(i1, 0, 1.0f);
    tripletListGj[3] = Tripletf(i2, 1, 1.0f);

    SparseMatf Gj(n_verts, 2);
    Gj.setFromTriplets(tripletListGj.begin(), tripletListGj.end());
    SparseMatf dx_ref_sp = Eigen::SparseView(dx_ref_mat);
    Gj = Gj * dx_ref_sp;

    SparseMatf SjT(2, 2 * n_faces);
    SjT.insert(0, 2 * j) = 1.0f;
    SjT.insert(1, 2 * j + 1) = 1.0f;

    L += face_mass(j) * (stiffness_hydro + stiffness_devia) * Gj *
         Gj.transpose();
    J += face_mass(j) * (stiffness_hydro + stiffness_devia) * Gj * SjT;
  }
  ratio = stiffness_devia / (stiffness_hydro + stiffness_devia);

  for (int i = 0; i < n_verts; i++) {
    M_h2.insert(i, i) = vert_mass(i) / dt / dt;
  }

  LHS_sparse = M_h2 + L;
  LHS = LHS_sparse;
  // solver = LHS.partialPivLu();
  // precompute LHS decomposition
  // solver.compute(LHS);
  sparse_solver.analyzePattern(LHS_sparse);
  sparse_solver.factorize(LHS_sparse);
}

template <>
void ProjectiveDynamicsSolver<2>::localStep(MatxXf& verts,
                                            const Matx3i& faces) {
  for (int j = 0; j < n_faces; j++) {
    int i0 = faces(j, 0);
    int i1 = faces(j, 1);
    int i2 = faces(j, 2);
    Vecxf dx1 = verts.row(i1) - verts.row(i0);
    Vecxf dx2 = verts.row(i2) - verts.row(i0);
    MatxXf F(2, 2);
    F << dx1, dx2;
    F = F * dx_ref_inv[j];

    Eigen::JacobiSVD<MatxXf> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    MatxXf U = svd.matrixU();
    MatxXf V = svd.matrixV();
    Vecxf S = svd.singularValues();

    float sig11 = S(0);
    float sig22 = S(1);
    float d1, d2;
    d1 = 1.0 - sig11;
    d2 = 1.0 - sig22;
    for (int iter = 0; iter < 20; iter++) {
      // float lambda = -d2 * (d2 + sig22);
      // float d1_new = -lambda * (d2 + sig22);
      float d1_new = d2 * (d2 + sig22) * (d2 + sig22);
      float d2_new = 1.0f / (d1_new + sig11) - sig22;
      if (fabs(d1_new - d1) < 1e-6 && fabs(d2_new - d2) < 1e-6) {
        break;
      }
      d1 = d1_new;
      d2 = d2_new;
    }
    Vecxf S_new(2);
    S_new << d1 + sig11, d2 + sig22;
    MatxXf D = U * S_new.asDiagonal() * V.transpose();
    ssvd<2>(U, S, V);
    MatxXf R = U * V.transpose();
    P.block(j * 2, 0, 2, 2) = (1.0f - ratio) * D + ratio * R;
  }
}

template <>
void ProjectiveDynamicsSolver<2>::globalStep(MatxXf& verts,
                                             const MatxXf& verts_pred) {
  Eigen::MatrixXf rhs = M_h2 * verts_pred + J * P;
  Eigen::MatrixXf result = sparse_solver.solve(rhs);
  // float error = (LHS * result - rhs).norm();
  // std::cout << "error: " << error << std::endl;
  verts = result;
}

ControlledProjDynSolver::ControlledProjDynSolver(
    ProjectiveDynamicsSolver<2>* proj_solver, MatxXf control_weights) {
  pd_solver = proj_solver;
  n_controls = control_weights.rows();
  SparseMatf LHS;
  LHS = pd_solver->LHS_sparse;
  int n_verts = LHS.rows();
  assert(control_weights.cols() == n_verts);
  int n_controls = control_weights.rows();
  SparseMatf combined_LHS(n_verts + n_controls, n_verts + n_controls);
  SparseMatf control_weights_sparse = control_weights.sparseView();
  SparseMatf control_weights_sparse_T;
  control_weights_sparse_T = control_weights_sparse.transpose();
  for (int k = 0; k < LHS.outerSize(); ++k)
    for (SparseMatf::InnerIterator it(LHS, k); it; ++it)
      combined_LHS.insert(it.row(), it.col()) = it.value();
  for (int i = 0; i < control_weights.rows(); i++)
    for (int j = 0; j < control_weights.cols(); j++) {
      combined_LHS.insert(n_verts + i, j) = control_weights(i, j);
      combined_LHS.insert(j, n_verts + i) = control_weights(i, j);
    }
  sparse_solver.analyzePattern(combined_LHS);
  sparse_solver.factorize(combined_LHS);
}

void ControlledProjDynSolver::globalStep(MatxXf& verts,
                                         const MatxXf& verts_pred,
                                         const MatxXf& control_verts,
                                         MatxXf& lambda) {
  Eigen::MatrixXf rhs =
      pd_solver->M_h2 * verts_pred + pd_solver->J * pd_solver->P;
  Eigen::MatrixXf control_rhs(n_controls + pd_solver->n_verts, 2);
  control_rhs.topRows(pd_solver->n_verts) = rhs;
  control_rhs.bottomRows(n_controls) = control_verts;
  Eigen::MatrixXf result = sparse_solver.solve(control_rhs);
  verts = result.topRows(pd_solver->n_verts);
  lambda = result.bottomRows(n_controls);
}

}  // namespace aphys