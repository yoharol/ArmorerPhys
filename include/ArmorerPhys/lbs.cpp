#include "ArmorerPhys/lbs.h"
#include "ArmorerPhys/math.h"

#include "igl/bbw.h"
#include "igl/boundary_conditions.h"

namespace aphys {

template <int dim>
void compress_lbs_weight(const MatxXd& W, const MatxXd& verts_ref,
                         SparseMatd& weight_mat) {
  int n_verts = W.rows();
  int n_bones = W.cols();

  auto set_block = [&](int vdx, int bdx) {
    for (int d1 = 0; d1 < dim; d1++) {
      for (int d2 = 0; d2 < dim; d2++) {
        weight_mat.insert(vdx * dim + d1,
                          bdx * dim * (dim + 1) + d1 * (dim + 1) + d2) =
            W(vdx, bdx) * verts_ref(vdx, d2);
      }
      weight_mat.insert(vdx * dim + d1, bdx * dim * (dim + 1) + d1 * (dim + 1) +
                                            dim) = W(vdx, bdx);
    }
  };

  for (int i = 0; i < n_verts; i++) {
    for (int j = 0; j < n_bones; j++) {
      if (W(i, j) > 0) set_block(i, j);
    }
  }
}

template void compress_lbs_weight<2>(const MatxXd& W, const MatxXd& verts_ref,
                                     SparseMatd& weight_mat);
template void compress_lbs_weight<3>(const MatxXd& W, const MatxXd& verts_ref,
                                     SparseMatd& weight_mat);

template <int dim>
void lbs(MatxXd& rig_verts, const SparseMatd& weight_mat, const Vecxd& T_vec) {
  Vecxd v = weight_mat * T_vec;
  rig_verts = MatxXd::Map(v.data(), v.size() / dim, dim);
}

template void lbs<2>(MatxXd& rig_verts, const SparseMatd& weight_mat,
                     const Vecxd& T_vec);
template void lbs<3>(MatxXd& rig_verts, const SparseMatd& weight_mat,
                     const Vecxd& T_vec);

template <int dim>
void construct_T_vec(const std::vector<MatxXd>& T, Vecxd& T_vec) {
  int n_bones = T.size();
  for (int bdx = 0; bdx < n_bones; bdx++) {
    Vecxd vec = Vecxd::Map(T[bdx].data(), dim * (dim + 1));
    T_vec.segment(bdx * dim * (dim + 1), dim * (dim + 1)) = vec;
  }
}

template void construct_T_vec<2>(const std::vector<MatxXd>& T, Vecxd& T_vec);
template void construct_T_vec<3>(const std::vector<MatxXd>& T, Vecxd& T_vec);

template <int dim>
void initialize_T_mat(int n, std::vector<MatxXd>& T) {
  T.clear();
  for (int i = 0; i < n; i++) {
    MatxXd T_i(dim, dim + 1);
    T_i.block(0, 0, dim, dim) = MatxXd::Identity(dim, dim);
    T_i.col(dim).setZero();
    T.emplace_back(T_i);
  }
}

template void initialize_T_mat<2>(int n, std::vector<MatxXd>& T);
template void initialize_T_mat<3>(int n, std::vector<MatxXd>& T);

template <int dim>
void construct_T_mat(const Vecxd& T_vec, std::vector<MatxXd>& T) {
  int n_bones = T.size();
  for (int i = 0; i < n_bones; i++) {
    T[i] = MatxXd::Map(T_vec.data() + i * dim * (dim + 1), dim, dim + 1);
  }
}

template void construct_T_mat<2>(const Vecxd& T_vec, std::vector<MatxXd>& T);
template void construct_T_mat<3>(const Vecxd& T_vec, std::vector<MatxXd>& T);

template <int dim>
void rotation_extraction(std::vector<MatxXd>& T) {
  for (int i = 0; i < T.size(); i++) {
    MatxXd A = T[i].block(0, 0, dim, dim);
    T[i].block(0, 0, dim, dim) = rotation_extraction(A);
  }
}

template void rotation_extraction<2>(std::vector<MatxXd>& T);
template void rotation_extraction<3>(std::vector<MatxXd>& T);

template <int dim>
bool bounded_biharmonic_weights(const MatxXd& verts, const Matx3i& faces,
                                const MatxXd& handles,
                                const Vecxi& control_point_indices,
                                const MatxXi& handle_edges, MatxXd& W,
                                int max_iter, int verbose) {
  igl::BBWData bbw_data;

  bbw_data.active_set_params.max_iter = max_iter;
  bbw_data.verbosity = verbose;

  Eigen::MatrixXd bc;
  Eigen::VectorXi b;
  Eigen::MatrixXd V = verts;
  Eigen::MatrixXi F = faces;
  Eigen::MatrixXd C = handles;
  Eigen::MatrixXi BE = handle_edges;
  igl::boundary_conditions(V, F, C, control_point_indices, BE,
                           Eigen::MatrixXi(), Eigen::MatrixXi(), b, bc);
  if (!igl::bbw(V, F, b, bc, bbw_data, W)) {
    return false;
  }
  for (int i = 0; i < W.rows(); i++) {
    // for (int j = 0; j < W.cols(); j++)
    //   if (W(i, j) < 1e-3) W(i, j) = 0;
    W.row(i) /= W.row(i).sum();
  }
  return true;
}

}  // namespace aphys
