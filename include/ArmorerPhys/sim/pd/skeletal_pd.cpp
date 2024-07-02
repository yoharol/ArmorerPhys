#include "ArmorerPhys/sim/pd/skeletal_pd.h"

namespace aphys {

SkeletalProjectiveDynamics3D::SkeletalProjectiveDynamics3D(
    const MatxXd& verts, const MatxXd& verts_ref, const Matx4i& tets,
    const Vecxd& tet_mass, const Vecxd& vert_mass, const MatxXd& external_force,
    double dt, double stiffness_hydro, double stiffness_devia,
    const SparseMatd& weight_mat)
    : ProjectiveDynamicsSolver3D(verts, verts_ref, tets, tet_mass, vert_mass,
                                 external_force, dt, stiffness_hydro,
                                 stiffness_devia) {
  LHS_sparse = weight_mat.transpose() * LHS_sparse * weight_mat;

  CheckError(LHS_sparse.cols() == weight_mat.rows(),
             "Matrix size mismatch: LHS_sparse.cols() != weight_mat.rows()",
             LHS_sparse.cols(), weight_mat.rows());
  CheckError(LHS_sparse.rows() == weight_mat.rows(),
             "Matrix size mismatch: LHS_sparse.rows() != weight_mat.rows()",
             LHS_sparse.rows(), weight_mat.rows());

  sparse_solver.analyzePattern(LHS_sparse);
  sparse_solver.factorize(LHS_sparse);
}

void SkeletalProjectiveDynamics3D::globalStep(const SparseMatd& weight_mat,
                                              MatxXd& verts,
                                              const MatxXd& verts_pred) {
  Eigen::MatrixXd rhs = weight_mat.transpose() * (M_h2 * verts_pred + J * P);
  Eigen::MatrixXd result = sparse_solver.solve(rhs);
  verts = result;
}

SkeletalProjectiveDynamics2D::SkeletalProjectiveDynamics2D(
    const MatxXd& verts, const MatxXd& verts_ref, const Matx3i& faces,
    const Vecxd& face_mass, const Vecxd& vert_mass,
    const MatxXd& external_force, double dt, double stiffness_hydro,
    double stiffness_devia, const SparseMatd& weight_mat)
    : ProjectiveDynamicsSolver2D(verts, verts_ref, faces, face_mass, vert_mass,
                                 external_force, dt, stiffness_hydro,
                                 stiffness_devia) {
  LHS_sparse = weight_mat.transpose() * LHS_sparse * weight_mat;

  CheckError(LHS_sparse.cols() == weight_mat.rows(),
             "Matrix size mismatch: LHS_sparse.cols() != weight_mat.rows()",
             LHS_sparse.cols(), weight_mat.rows());
  CheckError(LHS_sparse.rows() == weight_mat.rows(),
             "Matrix size mismatch: LHS_sparse.rows() != weight_mat.rows()",
             LHS_sparse.rows(), weight_mat.rows());

  sparse_solver.analyzePattern(LHS_sparse);
  sparse_solver.factorize(LHS_sparse);
}

void SkeletalProjectiveDynamics2D::globalStep(const SparseMatd& weight_mat,
                                              MatxXd& verts,
                                              const MatxXd& verts_pred) {
  Eigen::MatrixXd rhs = weight_mat.transpose() * (M_h2 * verts_pred + J * P);
  Eigen::MatrixXd result = sparse_solver.solve(rhs);
  verts = result;
}

}  // namespace aphys
