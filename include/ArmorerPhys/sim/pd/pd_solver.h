#ifndef ARMORER_SIM_PD_SOLVER_H_
#define ARMORER_SIM_PD_SOLVER_H_

#include <vector>
#include <Eigen/Core>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/LU>
#include <iostream>

#include "ArmorerPhys/type.h"

namespace aphys {

// Data for projective dynamics:
//  verts: n_verts x 2
//  verts_ref: n_verts x 2
//  verts_cache: n_verts x 2
//  faces: n_faces x 3
//  face_mass: n_faces x 1
//  vert_mass: n_verts x 1
//  external_force: n_verts x 2
//  dt, stiffness
// .P: 2 n_faces x 2
//  L: n_verts x n_verts
//  J: n_verts x 2 n_faces
//  M_h2:  n_verts x n_verts (M / h^2)
// solve function: (L + M_h2) x = M_h2 * x_pred + J * P

struct ProjectiveDynamicsSolver2D {
  int n_verts;
  int n_faces;
  std::vector<MatxXd> dx_ref_inv;
  MatxXd P;
  SparseMatd L;
  SparseMatd J;
  SparseMatd M_h2;
  MatxXd LHS;
  SparseMatd LHS_sparse;
  double ratio;
  Eigen::SparseLU<SparseMatd, Eigen::COLAMDOrdering<int>> sparse_solver;

  ProjectiveDynamicsSolver2D(const MatxXd& verts, const MatxXd& verts_ref,
                             const Matx3i& faces, const Vecxd& face_mass,
                             const Vecxd& vert_mass,
                             const MatxXd& external_force, double dt,
                             double stiffness_hydro, double stffness_devia);
  void localStep(const MatxXd& verts, const Matx3i& faces);
  void globalStep(MatxXd& verts, const MatxXd& verts_pred);
};

struct ControlledProjDynSolver {
  ProjectiveDynamicsSolver2D* pd_solver;
  int n_controls;
  Eigen::SparseLU<SparseMatd> sparse_solver;

  ControlledProjDynSolver(ProjectiveDynamicsSolver2D* proj_solver,
                          MatxXd control_weights);
  void globalStep(MatxXd& verts, const MatxXd& verts_pred,
                  const MatxXd& control_verts, MatxXd& lambda);
};

template <int dim>
Eigen::Vector<double, dim> solve_volume_sig(
    const Eigen::Vector<double, dim>& sig, double& lambda);

// Data for projective dynamics:
//  verts: n_verts x 3
//  verts_ref: n_verts x 3
//  verts_cache: n_verts x 3
//  tets: n_tets x 4
//  tet_mass: n_tets x 1
//  vert_mass: n_verts x 1
//  external_force: n_verts x 3
//  dt, stiffness
// .P: 3 n_tets x 3
//  L: n_verts x n_verts
//  J: n_verts x 3 n_faces
//  M_h2:  n_verts x n_verts (M / h^2)
// solve function: (L + M_h2) x = M_h2 * x_pred + J * P

struct ProjectiveDynamicsSolver3D {
  int n_verts;
  int n_tets;
  std::vector<Mat3d> dx_ref_inv;
  MatxXd P;
  SparseMatd L;
  SparseMatd J;
  SparseMatd M_h2;
  MatxXd LHS;
  SparseMatd LHS_sparse;
  double ratio;
  Eigen::SparseLU<SparseMatd, Eigen::COLAMDOrdering<int>> sparse_solver;

  ProjectiveDynamicsSolver3D(const MatxXd& verts, const MatxXd& verts_ref,
                             const Matx4i& tets, const Vecxd& tet_mass,
                             const Vecxd& vert_mass,
                             const MatxXd& external_force, double dt,
                             double stiffness_hydro, double stiffness_devia);
  void localStep(const MatxXd& verts, const Matx4i& tets);
  void globalStep(MatxXd& verts, const MatxXd& verts_pred);
};

}  // namespace aphys

#endif  // ARMORER_SIM_PD_SOLVER_H_
