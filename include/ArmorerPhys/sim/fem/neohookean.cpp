#include "ArmorerPhys/sim/fem/neohookean.h"

#include <Eigen/Dense>

#include <iostream>

namespace aphys {

void diff_d_F_d_x(const Mat2d& B, Eigen::Matrix<double, 4, 6>& dFdx) {
  dFdx.setZero();

  dFdx(0, 0) = B(0, 0);
  dFdx(0, 2) = B(1, 0);
  dFdx(0, 4) = -B(0, 0) - B(1, 0);

  dFdx(1, 1) = B(0, 0);
  dFdx(1, 3) = B(1, 0);
  dFdx(1, 5) = -B(0, 0) - B(1, 0);

  dFdx(2, 0) = B(0, 1);
  dFdx(2, 2) = B(1, 1);
  dFdx(2, 4) = -B(0, 1) - B(1, 1);

  dFdx(3, 1) = B(0, 1);
  dFdx(3, 3) = B(1, 1);
  dFdx(3, 5) = -B(0, 1) - B(1, 1);
}

void diff_d_F_d_x(const Mat3d& B, MatxXd& dFdx) {}

void NeoHookeanFEM2D::project_B(const MatxXd& verts, const MatxXd& verts_ref,
                                const Matx3i& faces, MatxXd& B) {
  int n_faces = faces.rows();
  B.resize(2, 2 * n_faces);
  B.setZero();
  for (int i = 0; i < n_faces; i++) {
    Vec2d p0 = verts_ref.row(faces(i, 0)).transpose();
    Vec2d p1 = verts_ref.row(faces(i, 1)).transpose();
    Vec2d p2 = verts_ref.row(faces(i, 2)).transpose();
    Mat2d Bm;
    Bm.col(0) = p0 - p2;
    Bm.col(1) = p1 - p2;
    Mat2d Bm_inv = Bm.inverse();
    B.block<2, 2>(0, 2 * i) = Bm_inv;
  }
}

void NeoHookeanFEM2D::project_F(const MatxXd& verts, const Matx3i& faces,
                                const MatxXd& B, MatxXd& F) {
  int n_faces = faces.rows();
  F.resize(2, 2 * n_faces);
  F.setZero();
  for (int i = 0; i < n_faces; i++) {
    Vec2d p0 = verts.row(faces(i, 0)).transpose();
    Vec2d p1 = verts.row(faces(i, 1)).transpose();
    Vec2d p2 = verts.row(faces(i, 2)).transpose();
    Mat2d Dm;
    Dm.col(0) = p0 - p2;
    Dm.col(1) = p1 - p2;
    F.block<2, 2>(0, 2 * i) = Dm * B.block<2, 2>(0, 2 * i);
  }
}

double NeoHookeanFEM2D::Energy(const MatxXd& F, const Vecxd& face_volume,
                               double mu, double lambda) {
  double energy = 0;
  double alpha = 1.0 + mu / lambda - mu / (4.0 * lambda);
  int n_faces = face_volume.size();
  for (int i = 0; i < n_faces; i++) {
    Matx2d F_i = F.block<2, 2>(0, 2 * i);
    double Ic = F_i.squaredNorm();
    double det = F_i.determinant();
    double Psi = 0.5 * mu * (Ic - 3.0) +
                 0.5 * lambda * (det - alpha) * (det - alpha) -
                 0.5 * mu * log(Ic + 1.0);
    energy += face_volume(i) * Psi;
  }
  return energy;
}

void NeoHookeanFEM2D::Jacobian(const MatxXd& F, const MatxXd& B,
                               const Matx3i& faces, const Vecxd& face_volume,
                               Vecxd& J, double mu, double lambda) {
  int n_faces = face_volume.size();
  double alpha = 1.0 + mu / lambda - mu / (4.0 * lambda);
  J.resize(2 * n_faces);
  J.setZero();
  for (int i = 0; i < n_faces; i++) {
    Mat2d F_i = F.block<2, 2>(0, 2 * i);
    Mat2d B_i = B.block<2, 2>(0, 2 * i);

    double Ic = F_i.squaredNorm();
    Mat2d P_i = mu * (1.0 - 1.0 / (Ic + 1.0)) * F_i;

    double det = F_i.determinant();
    Mat2d par_det_par_F = det * F_i.inverse().transpose();
    P_i += lambda * (det - alpha) * par_det_par_F;
    Vec4d vec_P_i = Eigen::Map<Vec4d>(P_i.data());

    Eigen::Matrix<double, 4, 6> dFdx;
    diff_d_F_d_x(B_i, dFdx);

    Vecxd dPsi_dx = face_volume(i) * dFdx.transpose() * vec_P_i;

    int i0 = faces(i, 0);
    int i1 = faces(i, 1);
    int i2 = faces(i, 2);

    Vec2d f0 = dPsi_dx.segment<2>(0);
    Vec2d f1 = dPsi_dx.segment<2>(2);
    Vec2d f2 = dPsi_dx.segment<2>(4);

    J.segment<2>(2 * i0) += f0;
    J.segment<2>(2 * i1) += f1;
    J.segment<2>(2 * i2) += f2;
  }
}
void NeoHookeanFEM2D::Hessian(const MatxXd& F, const MatxXd& B,
                              const Matx3i& faces, const Vecxd& face_volume,
                              MatxXd& H, double mu, double lambda) {
  int n_faces = faces.rows();
  double alpha = 1.0 + mu / lambda - mu / (4.0 * lambda);
  H.resize(2 * n_faces, 2 * n_faces);
  H.setZero();
  for (int i = 0; i < n_faces; i++) {
    Mat2d F_i = F.block<2, 2>(0, 2 * i);
    Mat2d B_i = B.block<2, 2>(0, 2 * i);
    double Ic = F_i.squaredNorm();
    double det = F_i.determinant();
    Mat2d par_det_par_F = det * F_i.inverse().transpose();

    Vec4d vec_F = Eigen::Map<Vec4d>(F_i.data(), 4);
    Vec4d vec_par_det_par_F = Eigen::Map<Vec4d>(par_det_par_F.data(), 4);

    Mat4d vec_T = Mat4d::Identity();
    vec_T = mu * (1.0 - 1.0 / (Ic + 1.0)) * vec_T;
    Mat4d vec_M = vec_F * vec_F.transpose();
    vec_M = mu * 2.0 / ((Ic + 1.0) * (Ic + 1.0)) * vec_M;
    Mat4d vec_G = vec_par_det_par_F * vec_par_det_par_F.transpose();
    vec_G = lambda * vec_G;

    Mat4d vec_H = Mat4d::Zero();
    vec_H(0, 3) = 1.0;
    vec_H(1, 2) = -1.0;
    vec_H(2, 1) = -1.0;
    vec_H(3, 0) = 1.0;
    vec_H = lambda * (det - alpha) * vec_H;

    Mat4d vec_FF = vec_T + vec_M + vec_G + vec_H;

    Eigen::Matrix<double, 4, 6> dFdx;
    diff_d_F_d_x(B_i, dFdx);

    // 6x6
    Eigen::Matrix<double, 6, 6> ddpsi_ddx =
        face_volume(i) * dFdx.transpose() * vec_FF * dFdx;

    int i0 = faces(i, 0);
    int i1 = faces(i, 1);
    int i2 = faces(i, 2);

    H.block<2, 2>(i0 * 2, i0 * 2) = ddpsi_ddx.block<2, 2>(0, 0);
    H.block<2, 2>(i0 * 2, i1 * 2) = ddpsi_ddx.block<2, 2>(0, 2);
    H.block<2, 2>(i0 * 2, i2 * 2) = ddpsi_ddx.block<2, 2>(0, 4);
    H.block<2, 2>(i1 * 2, i0 * 2) = ddpsi_ddx.block<2, 2>(2, 0);
    H.block<2, 2>(i1 * 2, i1 * 2) = ddpsi_ddx.block<2, 2>(2, 2);
    H.block<2, 2>(i1 * 2, i2 * 2) = ddpsi_ddx.block<2, 2>(2, 4);
    H.block<2, 2>(i2 * 2, i0 * 2) = ddpsi_ddx.block<2, 2>(4, 0);
    H.block<2, 2>(i2 * 2, i1 * 2) = ddpsi_ddx.block<2, 2>(4, 2);
    H.block<2, 2>(i2 * 2, i2 * 2) = ddpsi_ddx.block<2, 2>(4, 4);
  }
}

}  // namespace aphys