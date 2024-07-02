#include "ArmorerPhys/sim/common.h"

#include <Eigen/Dense>

#include <vector>
#include <algorithm>
#include <iostream>
#include <functional>

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/math.h"

namespace aphys {

void compute_mesh_mass(const MatxXd& verts, const Matx3i& faces,
                       Vecxd& face_mass, Vecxd& vert_mass, double rho) {
  int n_faces = faces.rows();
  int n_verts = verts.rows();
  face_mass.resize(n_faces);
  vert_mass.resize(n_verts);
  vert_mass.setZero();
  for (int i = 0; i < n_faces; i++) {
    int i1 = faces(i, 0);
    int i2 = faces(i, 1);
    int i3 = faces(i, 2);
    Vecxd v1 = verts.row(i1);
    Vecxd v2 = verts.row(i2) - v1.transpose();
    Vecxd v3 = verts.row(i3) - v1.transpose();
    double area = abs(computeArea(v2, v3) * rho);
    face_mass(i) = area;
    vert_mass(i1) += area / 3.0f;
    vert_mass(i2) += area / 3.0f;
    vert_mass(i3) += area / 3.0f;
  }
}

void compute_tet_mass(const MatxXd& verts, const Matx4i& tets, Vecxd& tet_mass,
                      Vecxd& vert_mass, double rho) {
  int n_tets = tets.rows();
  int n_verts = verts.rows();
  tet_mass.resize(n_tets);
  vert_mass.resize(n_verts);
  vert_mass.setZero();
  for (int i = 0; i < n_tets; i++) {
    int i0 = tets(i, 0);
    int i1 = tets(i, 1);
    int i2 = tets(i, 2);
    int i3 = tets(i, 3);
    Vecxd v0 = verts.row(i0);
    Vecxd v1 = verts.row(i1);
    Vecxd v2 = verts.row(i2);
    Vecxd v3 = verts.row(i3);
    double volume = abs(computeVolume(v1 - v0, v2 - v0, v3 - v0) * rho);
    tet_mass(i) = volume;
    vert_mass(i0) += volume / 4.0f;
    vert_mass(i1) += volume / 4.0f;
    vert_mass(i2) += volume / 4.0f;
    vert_mass(i3) += volume / 4.0f;
  }
}

void generate_gravity_force(const Vecxd& gravity, const Vecxd& vert_mass,
                            MatxXd& gravity_force) {
  int n_verts = vert_mass.size();
  int dim = gravity.size();
  gravity_force.resize(n_verts, dim);
  for (int i = 0; i < n_verts; i++) {
    gravity_force.row(i) = vert_mass(i) * gravity.transpose();
  }
}

void concatenate_add(Vecxd& A, const MatxXd& B, double scale) {
  assert(A.size() == B.rows() * B.cols());
  int dim = B.cols();
  for (int i = 0; i < B.rows(); i++) {
    A.segment(i, dim) += scale * B.row(i).transpose();
  }
}
void concatenate_add(MatxXd& A, const Vecxd& B, double scale) {
  assert(B.size() == A.rows() * A.cols());
  int dim = A.cols();
  for (int i = 0; i < A.rows(); i++) {
    A.row(i) += scale * B.segment(i * dim, dim).transpose();
  }
}

void concatenate_set(Vecxd& A, const MatxXd& B, double scale) {
  assert(A.size() == B.rows() * B.cols());
  int dim = B.cols();
  for (int i = 0; i < B.rows(); i++) {
    A.segment(i, dim) = scale * B.row(i).transpose();
  }
}

void concatenate_set(MatxXd& A, const Vecxd& B, double scale) {
  assert(B.size() == A.rows() * A.cols());
  int dim = A.cols();
  for (int i = 0; i < A.rows(); i++) {
    A.row(i) = scale * B.segment(i * dim, dim).transpose();
  }
}

double concatenate_dot(const MatxXd& A, const Vecxd& x, double scale) {
  assert(x.size() == A.rows() * A.cols());
  int dim = A.cols();
  double result = 0.0f;
  for (int i = 0; i < A.rows(); i++) {
    result += scale * A.row(i).dot(x.segment(i * dim, dim));
  }
  return result;
}

void ImplicitEuler::predict(MatxXd& predict, const MatxXd& verts,
                            const MatxXd& vel, const MatxXd& external_force,
                            const Vecxd& mass, double dt) {
  int n_verts = verts.rows();
  int dim = verts.cols();
  for (int i = 0; i < n_verts; i++) {
    predict.row(i) = verts.row(i) + dt * vel.row(i) +
                     dt * dt / mass(i) * external_force.row(i);
  }
}

double ImplicitEuler::modifyEnergy(double energy, const MatxXd& verts,
                                   const MatxXd& p, const Vecxd& mass,
                                   double dt) {
  int n_verts = verts.rows();
  for (int i = 0; i < n_verts; i++) {
    energy +=
        mass(i) / 2.0f / dt / dt * (verts.row(i) - p.row(i)).squaredNorm();
  }
  return energy;
}

// J_g = M / h^2 (x - p) + J
void ImplicitEuler::modifyJacobian(Vecxd& J, const MatxXd& verts,
                                   const MatxXd& p, const Vecxd& mass,
                                   double dt) {
  int n_verts = verts.rows();
  int dim = verts.cols();
  for (int i = 0; i < n_verts; i++) {
    J.segment(i * dim, dim) += mass(i) / dt / dt * (verts.row(i) - p.row(i));
  }
}

// H_g = M / h^2 I + H
void ImplicitEuler::modifyHessian(MatxXd& H, const Vecxd& mass, double dt) {
  int n_verts = mass.size();
  int dim = H.cols() / n_verts;
  for (int i = 0; i < n_verts; i++) {
    H.block(i * dim, i * dim, dim, dim) +=
        mass(i) / dt / dt * MatxXd::Identity(dim, dim);
  }
}

void ImplicitEuler::updateVelocity(MatxXd& vel, const MatxXd& verts,
                                   const MatxXd& verts_cache, double dt,
                                   double damping) {
  int n_verts = verts.rows();
  for (int i = 0; i < n_verts; i++) {
    vel.row(i) = (verts.row(i) - verts_cache.row(i)) / dt * exp(-damping * dt);
  }
}

void ImplicitEuler::solveDeltaVelocity(MatxXd& delta_vel,                //
                                       const MatxXd& verts,              //
                                       const MatxXd& external_force,     //
                                       const Vecxd& mass,                //
                                       const Vecxd& J, const MatxXd& H,  //
                                       const double dt) {
  int n_verts = verts.rows();
  int dim = verts.cols();
  Vecxd inv_mass = mass.cwiseInverse();
  DiagMatxXd inv_M;
  set_diag_matrix(inv_mass, inv_M, dim);
  MatxXd LHS =
      MatxXd::Identity(n_verts * dim, n_verts * dim) - dt * dt * inv_M * H;
  Vecxd verts_vec = Vecxd::Map(verts.data(), n_verts * dim);
  Vecxd extf_vec = Vecxd::Map(external_force.data(), n_verts * dim);
  MatxXd RHS = dt * inv_M * (extf_vec - J + dt * H * verts_vec);
  Vecxd dv_vec = LHS.colPivHouseholderQr().solve(RHS);
  delta_vel = MatxXd::Map(dv_vec.data(), n_verts, dim);
}

// line search along direction dv to minimize energy_func(v+alpha dv)
void line_search_mat(MatxXd& v, MatxXd& v_solver, const Vecxd& dv,
                     const Vecxd& J, EnergyFuncMatBased energy_func,
                     double beta, double gamma) {
  double alpha = 1.0f;
  v_solver = v;
  concatenate_add(v_solver, alpha * dv);
  double ddv = dv.transpose() * J;
  int iter = 0;
  double baseline = energy_func(v);
  while (energy_func(v_solver) > baseline + gamma * alpha * ddv) {
    alpha *= beta;
    v_solver = v;
    concatenate_add(v_solver, alpha * dv);
    if (iter++ > 100) {
      std::cout << "line search failed" << std::endl;
      break;
    }
  }
  v = v_solver;
}

void line_search_vec(Vecxd& v, Vecxd& v_solver, const Vecxd& dv, const Vecxd& J,
                     EnergyFuncVecBased energy_func, double beta,
                     double gamma) {
  double alpha = 1.0f;
  v_solver = v;
  v_solver += alpha * dv;
  double ddv = dv.transpose() * J;
  int iter = 0;
  double baseline = energy_func(v);
  while (energy_func(v_solver) > baseline + gamma * alpha * ddv) {
    alpha *= beta;
    v_solver = v;
    concatenate_add(v_solver, alpha * dv);
    if (iter++ > 100) {
      std::cout << "line search failed" << std::endl;
      break;
    }
  }
  v = v_solver;
}

}  // namespace aphys
