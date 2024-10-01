#include "ArmorerPhys/sim/pd/static_pd.h"
#include "ArmorerPhys/sim/fem.h"
#include "ArmorerPhys/glmath.h"

#include <iostream>

namespace aphys {

ARAPInterpolate2D::ARAPInterpolate2D(
    const MatxXd& verts, const MatxXd& verts_ref, const Matx3i& faces,
    const Vecxd& face_mass, const Vecxd& vert_mass,
    const MatxXd& external_force, double stiffness_hydro,
    double stiffness_devia, const Vecxi& fixed_verts)
    : solver(verts, verts_ref, faces, face_mass, vert_mass, external_force, 1.0,
             stiffness_hydro, stiffness_devia) {
  solver.LHS_sparse = solver.L;
  solver.LHS_sparse.conservativeResize(
      solver.LHS_sparse.rows() + fixed_verts.size(),
      solver.LHS_sparse.cols() + fixed_verts.size());
  for (int i = 0; i < fixed_verts.size(); i++) {
    solver.LHS_sparse.insert(solver.LHS_sparse.rows() - fixed_verts.size() + i,
                             fixed_verts[i]) = 1.0;
    solver.LHS_sparse.insert(fixed_verts[i], solver.LHS_sparse.cols() -
                                                 fixed_verts.size() + i) = 1.0;
  }
  solver.sparse_solver.analyzePattern(solver.LHS_sparse);
  solver.sparse_solver.factorize(solver.LHS_sparse);
}

void ARAPInterpolate2D::local_step(const MatxXd& verts, const Matx3i& faces,
                                   MatxXd& P) {
  solver.localStep(verts, faces);
  P = solver.P.transpose();
}

void ARAPInterpolate2D::solver_static_shape(MatxXd& verts, const Matx3i& faces,
                                            const MatxXd& fixed_pos,
                                            const MatxXd& P) {
  Eigen::MatrixXd F = P.transpose();
  Eigen::MatrixXd rhs = solver.J * F;
  rhs.conservativeResize(rhs.rows() + fixed_pos.rows(), rhs.cols());
  rhs.block(rhs.rows() - fixed_pos.rows(), 0, fixed_pos.rows(), rhs.cols()) =
      fixed_pos;
  Eigen::MatrixXd result = solver.sparse_solver.solve(rhs);
  verts = result.topRows(verts.rows());
}

ARAPTargetShape2D::ARAPTargetShape2D(const MatxXd& verts, const Matx3i& faces,
                                     const MatxXd& source_B) {
  int n_verts = verts.rows() / 2;
  int n_faces = faces.rows();

  CheckError(source_B.cols() == faces.rows() * 2,
             "source_B.cols() != n_faces * 2 ", source_B.rows(),
             faces.cols() * 2);
  CheckError(source_B.rows() == 2, "source_B.rows() != 2 ", source_B.cols());
  NeoHookeanFEM2D::project_F(verts, faces, source_B, F);
  S.resize(F.rows(), F.cols());
  rotate_angle.resize(n_faces);
  for (int i = 0; i < n_faces; i++) {
    MatxXd r, s;
    MatxXd f = F.block<2, 2>(0, 2 * i);
    polar_decomposition<2>(f, r, s);
    rotate_angle[i] = atan2(r(1, 0), r(0, 0));
    S.block<2, 2>(0, 2 * i) = s;
  }
}

void ARAPTargetShape2D::recover_deformation_map(const MatxXd& S,
                                                const Vecxd& rotate_angle,
                                                MatxXd& F) {
  CheckError(
      F.cols() == S.cols(),
      "[ArapTargetShape2D::recover_deformation_map] F.cols() != S.cols()",
      F.cols(), S.cols());
  CheckError(
      F.rows() == S.rows(),
      "[ArapTargetShape2D::recover_deformation_map] F.rows() != S.rows()",
      F.rows(), S.rows());

  int n_faces = rotate_angle.size();
  for (int i = 0; i < n_faces; i++) {
    MatxXd r(2, 2);
    r << cos(rotate_angle[i]), -sin(rotate_angle[i]), sin(rotate_angle[i]),
        cos(rotate_angle[i]);
    F.block<2, 2>(0, 2 * i) = r * S.block<2, 2>(0, 2 * i);
  }
}

}  // namespace aphys
