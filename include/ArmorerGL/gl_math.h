#ifndef GL_MATH_H_
#define GL_MATH_H_

#include <algorithm>
#include <Eigen/Core>
#include <vector>
#include <cassert>

namespace armgl {

Vec3f heat_rgb(float value, float minv, float maxv) {
  float ratio = 2 * (value - minv) / (maxv - minv);
  float b = std::max(0.0f, 1 - ratio);
  float r = std::max(0.0f, ratio - 1);
  float g = 1 - b - r;
  return Vec3f(r, g, b);
}

void Mat3fToList3f(const Mat3f& mat, List3f& vec) {
  assert(mat.cols() == 3);
  vec.resize(mat.rows());
  for (int i = 0; i < mat.rows(); ++i) {
    vec[i] = mat.row(i).transpose();
  }
}

void List3fToMat3f(const List3f& vec, Mat3f& mat) {
  mat.resize(vec.size(), 3);
  for (int i = 0; i < vec.size(); ++i) {
    mat.row(i) = vec[i].transpose();
  }
}

void List3fToList2f(const List3f& vec3, List2f& vec2) {
  vec2.resize(vec3.size());
  for (int i = 0; i < vec3.size(); ++i) {
    vec2[i] = vec3[i].head(2);
  }
}

void List2fToList3f(const List2f& vec2, List3f& vec3) {
  vec3.resize(vec2.size());
  for (int i = 0; i < vec2.size(); ++i) {
    vec3[i] = Vec3f(vec2[i](0), vec2[i](1), 0.0f);
  }
}

void Mat2fToList3f(const Mat2f& mat, List3f& vec) {
  assert(mat.cols() == 2);
  vec.resize(mat.rows());
  for (int i = 0; i < mat.rows(); ++i) {
    vec[i] = Vec3f(mat(i, 0), mat(i, 1), 0.0f);
  }
}

};  // namespace armgl

#endif  // GL_MATH_H_