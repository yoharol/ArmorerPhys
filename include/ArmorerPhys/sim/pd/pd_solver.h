#ifndef ARMORER_SIM_PD_SOLVER_H_
#define ARMORER_SIM_PD_SOLVER_H_

#include <vector>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/LU>
#include <iostream>

#include "ArmorerPhys/type.h"

namespace aphys {

// Data for projective dynamics:
//  verts: n_verts x 3
//  verts_ref: n_verts x 3
//  verts_cache: n_verts x 3
//  faces: n_faces x 3
//  face_mass: n_faces x 1
//  vert_mass: n_verts x 1
//  external_force: n_verts x 3
//  dt, stiffness
// .P: 2 n_faces x 2
//  L: n_verts x n_verts
//  J: n_verts x 2 n_faces
//  M_h2:  n_verts x n_verts (M / h^2)
// solve function: (L + M_h2) x = M_h2 * x_pred + J * P

template <int dim>
struct ProjectiveDynamicsSolver {
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

  ProjectiveDynamicsSolver(const MatxXd& verts, const MatxXd& verts_ref,
                           const Matx3i& faces, const Vecxd& face_mass,
                           const Vecxd& vert_mass, const MatxXd& external_force,
                           double dt, double stiffness_hydro,
                           double stffness_devia);
  void localStep(MatxXd& verts, const Matx3i& faces);
  void globalStep(MatxXd& verts, const MatxXd& verts_pred);
};

struct ControlledProjDynSolver {
  ProjectiveDynamicsSolver<2>* pd_solver;
  int n_controls;
  Eigen::SparseLU<SparseMatd> sparse_solver;

  ControlledProjDynSolver(ProjectiveDynamicsSolver<2>* proj_solver,
                          MatxXd control_weights);
  void globalStep(MatxXd& verts, const MatxXd& verts_pred,
                  const MatxXd& control_verts, MatxXd& lambda);
};

}  // namespace aphys

#endif  // ARMORER_SIM_PD_SOLVER_H_