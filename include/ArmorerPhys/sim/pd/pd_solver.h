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
  std::vector<MatxXf> dx_ref_inv;
  MatxXf P;
  SparseMatf L;
  SparseMatf J;
  SparseMatf M_h2;
  MatxXf LHS;
  SparseMatf LHS_sparse;
  float ratio;
  // Eigen::SparseLU<SparseMatf> solver;
  Eigen::PartialPivLU<MatxXf> solver;
  Eigen::SparseLU<SparseMatf, Eigen::COLAMDOrdering<int>> sparse_solver;

  ProjectiveDynamicsSolver(const MatxXf& verts, const MatxXf& verts_ref,
                           const Matx3i& faces, const Vecxf& face_mass,
                           const Vecxf& vert_mass, const MatxXf& external_force,
                           float dt, float stiffness_hydro,
                           float stffness_devia);
  void localStep(MatxXf& verts, const Matx3i& faces);
  void globalStep(MatxXf& verts, const MatxXf& verts_pred);
};

struct ControlledProjDynSolver {
  ProjectiveDynamicsSolver<2>* pd_solver;
  int n_controls;
  Eigen::SparseLU<SparseMatf> sparse_solver;

  ControlledProjDynSolver(ProjectiveDynamicsSolver<2>* proj_solver,
                          MatxXf control_weights);
  void globalStep(MatxXf& verts, const MatxXf& verts_pred,
                  const MatxXf& control_verts, MatxXf& lambda);
};

}  // namespace aphys

#endif  // ARMORER_SIM_PD_SOLVER_H_