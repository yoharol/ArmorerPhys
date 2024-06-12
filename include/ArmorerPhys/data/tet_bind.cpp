#include "ArmorerPhys/data/tet_bind.h"

#include <iostream>

#include "ArmorerPhys/geom.h"
#include "ArmorerPhys/tet.h"

namespace aphys {

void interpolate_barycentric(aphys::MatxXd& mesh, const aphys::MatxXd& verts,
                             const aphys::Matx4i& tets, const Vecxi& bind_index,
                             const MatxXd& bind_weights) {
  for (int i = 0; i < mesh.rows(); i++) {
    int idx = bind_index(i);
    Vec4i tet = tets.row(idx);
    Vecxd v0 = verts.row(tet(0));
    Vecxd v1 = verts.row(tet(1));
    Vecxd v2 = verts.row(tet(2));
    Vecxd v3 = verts.row(tet(3));
    Vec4d bary = bind_weights.row(i);
    mesh.row(i) =
        (bary(0) * v0 + bary(1) * v1 + bary(2) * v2 + bary(3) * v3).transpose();
  }
}

void bind_to_tet(const MatxXd& verts, const TetMesh& tm, aphys::Vecxi& bc_index,
                 aphys::MatxXd& bc_weights) {
  bc_index.resize(verts.rows());
  bc_weights.resize(verts.rows(), 4);
  for (int i = 0; i < verts.rows(); i++) {
    aphys::Vec3d p = verts.row(i);
    int tet_idx = -1;
    for (int j = 0; j < tm.tets.rows(); j++) {
      aphys::Vec3d p0 = tm.verts.row(tm.tets(j, 0));
      aphys::Vec3d p1 = tm.verts.row(tm.tets(j, 1));
      aphys::Vec3d p2 = tm.verts.row(tm.tets(j, 2));
      aphys::Vec3d p3 = tm.verts.row(tm.tets(j, 3));
      bool inside = aphys::inside_tet(p, p0, p1, p2, p3);
      if (inside) {
        tet_idx = j;
        bc_index(i) = j;
        aphys::Vec4d bary;
        aphys::compute_barycentric_tet(p, p0, p1, p2, p3, bary);
        bc_weights.row(i) = bary.transpose();
        break;
      }
    }
    if (tet_idx == -1) {
      std::cout << "point not in tet" << std::endl;
    }
  }
}

void generate_bind_mat(const int n_tet_verts, const Matx4i& tets,
                       const Vecxi& bc_index, const MatxXd& bc_weights,
                       SparseMatd& bind_mat) {
  int n_verts = bc_index.size();
  bind_mat.resize(n_verts, n_tet_verts);
  bind_mat.reserve(n_verts * 4);
  for (int i = 0; i < n_verts; i++) {
    bind_mat.insert(i, tets(bc_index(i), 0)) = bc_weights(i, 0);
    bind_mat.insert(i, tets(bc_index(i), 1)) = bc_weights(i, 1);
    bind_mat.insert(i, tets(bc_index(i), 2)) = bc_weights(i, 2);
    bind_mat.insert(i, tets(bc_index(i), 3)) = bc_weights(i, 3);
  }
  bind_mat.makeCompressed();
}

StaticTetBindPDSolver::StaticTetBindPDSolver(
    const TetMesh& tm, const MatxXd& verts_ref, const Vecxd& vert_mass,
    const Vecxd& tet_mass, const MatxXd& external_force, double stiffness_hydro,
    double stiffness_devia, const SparseMatd& bind_mat)
    : solver(tm.verts, verts_ref, tm.tets, tet_mass, vert_mass, external_force,
             1.0, stiffness_hydro, stiffness_devia) {
  SparseMatd WTW = bind_mat.transpose() * bind_mat;
  solver.LHS_sparse = solver.L + WTW;
  solver.sparse_solver.analyzePattern(solver.LHS_sparse);
  solver.sparse_solver.factorize(solver.LHS_sparse);
}

void StaticTetBindPDSolver::local_step(const MatxXd& verts,
                                       const Matx4i& tets) {
  solver.localStep(verts, tets);
}

void StaticTetBindPDSolver::global_step(MatxXd& verts, const MatxXd& mesh_verts,
                                        const SparseMatd& bind_mat) {
  Eigen::MatrixXd rhs = solver.J * solver.P + bind_mat.transpose() * mesh_verts;
  Eigen::MatrixXd result = solver.sparse_solver.solve(rhs);
  verts = result;
}

}  // namespace aphys
