#include "ArmorerPhys/sim/pd/pd_solver.h"

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SVD>
#include <Eigen/IterativeLinearSolvers>

#include <stdexcept>
#include <iostream>

#include "ArmorerPhys/math.h"

namespace aphys {

ProjectiveDynamicsSolver3D::ProjectiveDynamicsSolver3D(
    const MatxXd& verts, const MatxXd& verts_ref, const Matx4i& tets,
    const Vecxd& tet_mass, const Vecxd& vert_mass, const MatxXd& external_force,
    double dt, double stiffness_hydro, double stiffness_devia) {
  n_verts = verts.rows();
  n_tets = tets.rows();
  L.resize(n_verts, n_verts);
  J.resize(n_verts, 3 * n_tets);
  M_h2.resize(n_verts, n_verts);
  LHS.resize(n_verts, n_verts);
  dx_ref_inv.resize(n_tets);
  P.resize(n_tets * 3, 3);

  MatxXd L_mat(n_verts, n_verts);
  L_mat.setZero();
  MatxXd J_mat(n_verts, 3 * n_tets);
  J_mat.setZero();

  for (int j = 0; j < n_tets; j++) {
    int i0 = tets(j, 0);
    int i1 = tets(j, 1);
    int i2 = tets(j, 2);
    int i3 = tets(j, 3);

    Vec3d dx1_ref = verts_ref.row(i1) - verts_ref.row(i0);
    Vec3d dx2_ref = verts_ref.row(i2) - verts_ref.row(i0);
    Vec3d dx3_ref = verts_ref.row(i3) - verts_ref.row(i0);
    MatxXd dx_ref_mat(3, 3);
    dx_ref_mat << dx1_ref, dx2_ref, dx3_ref;
    dx_ref_mat = dx_ref_mat.inverse();
    dx_ref_inv[j] = dx_ref_mat;

    std ::vector<Tripletd> tripletListGj(6);
    tripletListGj[0] = Tripletd(i0, 0, -1.0);
    tripletListGj[1] = Tripletd(i0, 1, -1.0);
    tripletListGj[2] = Tripletd(i0, 2, -1.0);
    tripletListGj[3] = Tripletd(i1, 0, 1.0);
    tripletListGj[4] = Tripletd(i2, 1, 1.0);
    tripletListGj[5] = Tripletd(i3, 2, 1.0);

    SparseMatd Gj(n_verts, 3);
    Gj.setFromTriplets(tripletListGj.begin(), tripletListGj.end());
    SparseMatd dx_ref_sp = Eigen::SparseView(dx_ref_mat);
    Gj = Gj * dx_ref_sp;

    SparseMatd SjT(3, 3 * n_tets);
    SjT.insert(0, 3 * j) = 1.0;
    SjT.insert(1, 3 * j + 1) = 1.0;
    SjT.insert(2, 3 * j + 2) = 1.0;

    SparseMatd test = Gj * Gj.transpose();

    L_mat +=
        tet_mass(j) * (stiffness_hydro + stiffness_devia) * Gj * Gj.transpose();
    J_mat += tet_mass(j) * (stiffness_hydro + stiffness_devia) * Gj * SjT;
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

void ProjectiveDynamicsSolver3D::localStep(const MatxXd& verts,
                                           const Matx4i& tets) {
  MatxXd U, V;
  Vecxd S;
  MatxXd F(3, 3);
  MatxXd lhs(4, 4);
  Vecxd rhs(4);
  Vecxd delta(4);
  for (int j = 0; j < n_tets; j++) {
    int i0 = tets(j, 0);
    int i1 = tets(j, 1);
    int i2 = tets(j, 2);
    int i3 = tets(j, 3);
    Vecxd dx1 = verts.row(i1) - verts.row(i0);
    Vecxd dx2 = verts.row(i2) - verts.row(i0);
    Vecxd dx3 = verts.row(i3) - verts.row(i0);
    F << dx1, dx2, dx3;
    F = F * dx_ref_inv[j];

    Eigen::JacobiSVD<MatxXd> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();
    ssvd<3>(U, S, V);

    double sig11 = S(0);
    double sig22 = S(1);
    double sig33 = S(2);

    double d1, d2, d3;
    d1 = 1.0 - sig11;
    d2 = 1.0 - sig22;
    d3 = 1.0 - sig33;

    // solve
    double lambda = 0.0f;
    for (int iter = 0; iter < 20; iter++) {
      double p1 = d1 + sig11;
      double p2 = d2 + sig22;
      double p3 = d3 + sig33;

      lhs << 1.0f, lambda * p3, lambda * p2, p2 * p3,  //
          lambda * p3, 1.0f, lambda * p1, p1 * p3,     //
          lambda * p2, lambda * p1, 1.0, p1 * p2,      //
          p2 * p3, p1 * p3, p1 * p2, 0.0f;             //
      rhs << d1 + lambda * p2 * p3,                    //
          d2 + lambda * p1 * p3,                       //
          d3 + lambda * p1 * p2,                       //
          p1 * p3 * p3 - 1.0f;
      rhs = -rhs;
      delta = lhs.partialPivLu().solve(rhs);
      if ((delta.squaredNorm()) < 1e-6) break;
      d1 += delta(0);
      d2 += delta(1);
      d3 += delta(2);
      lambda += delta(3);
    }

    Vecxd S_new(3);
    S_new << d1 + sig11, d2 + sig22, d3 + sig33;
    MatxXd D = U * S_new.asDiagonal() * V.transpose();
    MatxXd R = U * V.transpose();
    P.block(j * 3, 0, 3, 3) = (1.0f - ratio) * D + ratio * R;
  }
}

void ProjectiveDynamicsSolver3D::globalStep(MatxXd& verts,
                                            const MatxXd& verts_pred) {
  Eigen::MatrixXd rhs = M_h2 * verts_pred + J * P;
  Eigen::MatrixXd result = sparse_solver.solve(rhs.cast<double>());
  verts = result.cast<double>();
}

}  // namespace aphys
