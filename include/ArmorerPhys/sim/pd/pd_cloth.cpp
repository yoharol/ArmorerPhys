#include "ArmorerPhys/sim/pd/pd_cloth.h"

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SVD>
#include <Eigen/IterativeLinearSolvers>
#include <iostream>

#include "ArmorerPhys/glmath.h"
#include "ArmorerPhys/geom.h"

namespace aphys {

ProjectiveCloth::ProjectiveCloth(const ClothMesh& mesh, const double dt,
                                 const double bending_stiffness,
                                 const double stretching_stiffness)
    : bending_stiffness(bending_stiffness),
      stretching_stiffness(stretching_stiffness) {
  n_verts = mesh.n_verts;
  n_faces = mesh.n_faces;
  n_edges = mesh.n_edges;
  L.resize(n_verts, n_verts);
  J.resize(n_verts, 3 * n_faces);
  M_h2.resize(n_verts, n_verts);
  P.resize(2 * n_faces, 3);

  {
    MatxXd L_mat(n_verts, n_verts);
    L_mat.setZero();
    MatxXd J_mat(n_verts, 2 * n_faces);
    J_mat.setZero();

    for (int j = 0; j < n_faces; j++) {
      int i0 = mesh.faces(j, 0);
      int i1 = mesh.faces(j, 1);
      int i2 = mesh.faces(j, 2);
      Vecxd dx1 = mesh.verts.row(i1) - mesh.verts.row(i0);
      Vecxd dx2 = mesh.verts.row(i2) - mesh.verts.row(i0);
      MatxXd dx_ref(2, 2);
      dx_ref << dx1, dx2;
      MatxXd dx_inv = dx_ref.inverse();
      dx_ref_inv.push_back(dx_inv);

      std::vector<Tripletd> tripletListGj(4);
      tripletListGj[0] = Tripletd(i0, 0, -1.0);
      tripletListGj[1] = Tripletd(i0, 1, -1.0);
      tripletListGj[2] = Tripletd(i1, 0, 1.0);
      tripletListGj[3] = Tripletd(i2, 1, 1.0);
      SparseMatd Gj(n_verts, 2);
      Gj.setFromTriplets(tripletListGj.begin(), tripletListGj.end());
      SparseMatd dx_ref_sp = Eigen::SparseView(dx_inv);
      Gj = Gj * dx_ref_sp;

      SparseMatd SjT(2, 2 * n_faces);
      SjT.insert(0, 2 * j) = 1.0;
      SjT.insert(1, 2 * j + 1) = 1.0;

      L_mat += mesh.face_mass(j) * stretching_stiffness * Gj * Gj.transpose();
      J_mat += mesh.face_mass(j) * stretching_stiffness * Gj * SjT;
    }
    L = L_mat.sparseView();
    J = J_mat.sparseView();
  }

  {
    MatxXd bending_K_mat(n_edges, n_verts);
    bending_K_mat.setZero();
    for (int j = 0; j < n_edges; j++) {
      int i0 = mesh.edges(j, 0);
      int i1 = mesh.edges(j, 1);
      int i2 = mesh.edges(j, 2);
      int i3 = mesh.edges(j, 3);
      if (i3 == -1) continue;
      double l10 = (mesh.verts.row(i1) - mesh.verts.row(i0)).norm();
      double l20 = (mesh.verts.row(i2) - mesh.verts.row(i0)).norm();
      double l30 = (mesh.verts.row(i3) - mesh.verts.row(i0)).norm();
      double l21 = (mesh.verts.row(i2) - mesh.verts.row(i1)).norm();
      double l31 = (mesh.verts.row(i3) - mesh.verts.row(i1)).norm();

      // Heron's formula
      double s0 = (l10 + l20 + l21) / 2;
      double A0 = std::sqrt(s0 * (s0 - l10) * (s0 - l20) * (s0 - l21));
      double s1 = (l10 + l30 + l31) / 2;
      double A1 = std::sqrt(s1 * (s1 - l10) * (s1 - l30) * (s1 - l31));

      // https://cims.nyu.edu/gcl/papers/bergou2006qbm.pdf
      double cot01 = ((l10 * l10 + l20 * l20 - l21 * l21) / 4) / A0;
      double cot02 = ((l10 * l10 + l30 * l30 - l31 * l31) / 4) / A1;
      double cot03 = ((l10 * l10 + l21 * l21 - l20 * l20) / 4) / A0;
      double cot04 = ((l10 * l10 + l31 * l31 - l30 * l30) / 4) / A1;
      Vec4d edge_K;
      edge_K << cot03 + cot04, cot01 + cot02, -cot01 - cot03, -cot02 - cot04;
      edge_K *= sqrt(bending_stiffness * 3.0 / (A0 + A1));
      bending_K_mat(j, i0) = edge_K(0);
      bending_K_mat(j, i1) = edge_K(1);
      bending_K_mat(j, i2) = edge_K(2);
      bending_K_mat(j, i3) = edge_K(3);
    }
    SparseMatd bending_K = bending_K_mat.sparseView();
    B = bending_K.transpose() * bending_K;
  }

  for (int i = 0; i < n_verts; i++) {
    M_h2.insert(i, i) = mesh.verts_mass(i) / dt / dt;
  }
  M_h2.makeCompressed();

  LHS = M_h2 + L + B;
  sparse_solver.analyzePattern(LHS);
  sparse_solver.factorize(LHS);
}

void ProjectiveCloth::localStep(const MatxXd& verts, const ClothMesh& mesh) {
#pragma omp parallel for
  for (int j = 0; j < n_faces; j++) {
    int i0 = mesh.faces(j, 0);
    int i1 = mesh.faces(j, 1);
    int i2 = mesh.faces(j, 2);
    Vecxd dx1 = verts.row(i1) - verts.row(i0);
    Vecxd dx2 = verts.row(i2) - verts.row(i0);
    MatxXd F(3, 2);
    F << dx1, dx2;
    F = F * dx_ref_inv[j];

    Eigen::JacobiSVD<MatxXd> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    MatxXd U = svd.matrixU();
    MatxXd V = svd.matrixV();
    // Vecxd S = svd.singularValues();

    MatxXd T = U.leftCols(2) * V.transpose();

    P.block(j * 2, 0, 2, 3) = T.transpose();
  }
}

void ProjectiveCloth::globalStep(MatxXd& verts, const MatxXd& verts_pred) {
  Eigen::MatrixXd rhs = M_h2 * verts_pred + J * P;
  Eigen::MatrixXd result = sparse_solver.solve(rhs);
  verts = result;
}

}  // namespace aphys
