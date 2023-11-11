#ifndef ARMORER_USD_TYPES_H
#define ARMORER_USD_TYPES_H

#include <Eigen/Core>
#include <pxr/base/vt/value.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>

namespace aphys {

/*
 * Generate a VtArray from Eigen::VectorXf array
 */
template <typename T>
pxr::VtValue VtVecX(const Eigen::Matrix<T, Eigen::Dynamic, 1>& eigen_arr) {
  pxr::VtArray<T> vt_arr(eigen_arr.rows());
  for (int i = 0; i < eigen_arr.rows(); i++) vt_arr[i] = eigen_arr(i);
  return pxr::VtValue(vt_arr);
}

template <typename T>
pxr::VtValue VtVecXFlatten(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& eigen_mat) {
  pxr::VtArray<T> vt_arr(eigen_mat.rows() * eigen_mat.cols());
  for (int i = 0; i < eigen_mat.rows(); i++)
    for (int j = 0; j < eigen_mat.cols(); j++)
      vt_arr[i * eigen_mat.cols() + j] = eigen_mat(i, j);
  return pxr::VtValue(vt_arr);
}

/*
 * Generate a VtArray with size and value
 */
template <typename T>
pxr::VtValue VtVecX(int size, T value) {
  return pxr::VtValue(pxr::VtArray<T>(size, value));
}

/*
 * Generate a PxrType array from Eigen::MatrixXT
 */
template <typename PxrType, typename T>
pxr::VtValue VtMatxX(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& eigen_mat) {
  pxr::VtArray<PxrType> pxr_arr(eigen_mat.rows());
  assert(PxrType::dimension == eigen_mat.cols());
  for (int i = 0; i < eigen_mat.rows(); i++)
    for (int j = 0; j < eigen_mat.cols(); j++) pxr_arr[i][j] = eigen_mat(i, j);
  return pxr::VtValue(pxr_arr);
}

pxr::VtValue VtMat3Xf(const Eigen::MatrixX3f& eigen_mat) {
  return VtMatxX<pxr::GfVec3f, float>(eigen_mat);
}

void ReadVtMat3Xf(pxr::VtArray<pxr::GfVec3f>& vt_arr,
                  Eigen::MatrixX3f& eigen_mat) {
  eigen_mat.resize(vt_arr.size(), 3);
  for (int i = 0; i < vt_arr.size(); i++) {
    eigen_mat(i, 0) = vt_arr[i][0];
    eigen_mat(i, 1) = vt_arr[i][1];
    eigen_mat(i, 2) = vt_arr[i][2];
  }
}

}  // namespace aphys

#endif  // ARMORER_USD_TYPES_H