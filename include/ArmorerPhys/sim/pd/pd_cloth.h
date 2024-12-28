#ifndef ARMORER_SIM_PD_CLOTH_H_
#define ARMORER_SIM_PD_CLOTH_H_

#include <vector>
#include <Eigen/SparseLU>

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/data/mesh.h"

namespace aphys {

struct ProjectiveCloth {
  int n_verts;
  int n_faces;
  int n_edges;
  std::vector<MatxXd> dx_ref_inv;
  MatxXd P;
  SparseMatd L;
  SparseMatd J;
  SparseMatd M_h2;
  SparseMatd LHS;
  SparseMatd B;
  double bending_stiffness;
  double stretching_stiffness;
  Eigen::SparseLU<SparseMatd, Eigen::COLAMDOrdering<int>> sparse_solver;

  ProjectiveCloth(const ClothMesh& mesh, const double dt,
                  const double bending_stiffness,
                  const double stretching_stiffness);

  void localStep(const MatxXd& verts, const ClothMesh& mesh);
  void globalStep(MatxXd& verts, const MatxXd& verts_pred);
};

}  // namespace aphys

#endif  // ARMORER_SIM_PD_CLOTH_H_
