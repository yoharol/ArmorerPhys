#ifndef ARMORER_PHYS_LBS_H_
#define ARMORER_PHYS_LBS_H_

#include <vector>
#include "ArmorerPhys/type.h"

namespace aphys {

template <int dim>
void compress_lbs_weight(const MatxXd& W, const MatxXd& verts_ref,
                         SparseMatd& weight_mat);

template <int dim>
void lbs(MatxXd& rig_verts, const SparseMatd& weight_mat, const Vecxd& T_vec);

template <int dim>
void construct_T_vec(const std::vector<MatxXd>& T, Vecxd& T_vec);

template <int dim>
void construct_T_mat(const Vecxd& T_vec, std::vector<MatxXd>& T);

template <int dim>
void initialize_T_mat(int n, std::vector<MatxXd>& T);

template <int dim>
void rotation_extraction(std::vector<MatxXd>& T);

template <int dim>
bool bounded_biharmonic_weights(const MatxXd& verts, const Matx3i& faces,
                                const MatxXd& handles,
                                const Vecxi& control_point_indices,
                                const MatxXi& handle_edges, MatxXd& W,
                                int max_iter = 8, int verbose = 2);

}  // namespace aphys

#endif  // ARMORER_PHYS_LBS_H_
