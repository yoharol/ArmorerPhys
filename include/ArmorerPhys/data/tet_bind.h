#ifndef ARMORPHYS_DATA_TET_BIND_H_
#define ARMORPHYS_DATA_TET_BIND_H_

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/tet.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/pd.h"

namespace aphys {
void interpolate_barycentric(aphys::MatxXd& mesh, const aphys::MatxXd& verts,
                             const aphys::Matx4i& tets, const Vecxi& bind_index,
                             const MatxXd& bind_weights);

void bind_to_tet(const MatxXd& verts, const TetMesh& tm, aphys::Vecxi& bc_index,
                 aphys::MatxXd& bc_weights);

void generate_bind_mat(const int n_tet_verts, const Matx4i& tets,
                       const Vecxi& bc_index, const MatxXd& bc_weights,
                       SparseMatd& bind_mat);

struct StaticTetBindPDSolver {
  ProjectiveDynamicsSolver3D solver;

  StaticTetBindPDSolver(const TetMesh& tm, const MatxXd& verts_ref,
                        const Vecxd& vert_mass, const Vecxd& tet_mass,
                        const MatxXd& external_force, double stiffness_hydro,
                        double stiffness_devia, const SparseMatd& bind_mat);

  void local_step(const MatxXd& verts, const Matx4i& tets);
  void global_step(MatxXd& verts, const MatxXd& mesh_verts,
                   const SparseMatd& bind_mat);

  void solver_static_shape(const MatxXd& mesh_verts, TetMesh& tm);
};

}  // namespace aphys

#endif  // ARMORPHYS_DATA_TET_BIND_H_
