#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <cassert>
#include <cmath>

#include "ArmorerPhys/math.h"

namespace aphys {

Vec3f heat_rgb(float value, float minv, float maxv) {
  float ratio = 2 * (value - minv) / (maxv - minv);
  float b = std::max(0.0f, 1 - ratio);
  float r = std::max(0.0f, ratio - 1);
  float g = 1 - b - r;
  return Vec3f(r, g, b);
}

void Mat3fToList3f(const Matx3f& mat, Listx3f& vec) {
  assert(mat.cols() == 3);
  vec.resize(mat.rows());
  for (int i = 0; i < mat.rows(); ++i) {
    vec[i] = mat.row(i).transpose();
  }
}

void List3fToMat3f(const Listx3f& vec, Matx3f& mat) {
  mat.resize(vec.size(), 3);
  for (int i = 0; i < vec.size(); ++i) {
    mat.row(i) = vec[i].transpose();
  }
}

void List3fToList2f(const Listx3f& vec3, Listx2f& vec2) {
  vec2.resize(vec3.size());
  for (int i = 0; i < vec3.size(); ++i) {
    vec2[i] = vec3[i].head(2);
  }
}

void List2fToList3f(const Listx2f& vec2, Listx3f& vec3) {
  vec3.resize(vec2.size());
  for (int i = 0; i < vec2.size(); ++i) {
    vec3[i] = Vec3f(vec2[i](0), vec2[i](1), 0.0f);
  }
}

void Mat2fToList3f(const Matx2f& mat, Listx3f& vec) {
  assert(mat.cols() == 2);
  vec.resize(mat.rows());
  for (int i = 0; i < mat.rows(); ++i) {
    vec[i] = Vec3f(mat(i, 0), mat(i, 1), 0.0f);
  }
}

float computeArea(const Vecxf& vec1, const Vecxf& vec2) {
  if (vec1.size() != vec2.size()) {
    throw std::runtime_error("computeArea: vec1.size() != vec2.size()");
  }
  float area = 0.0f;
  if (vec1.size() == 2) {
    area = 0.5f * (vec1(0) * vec2(1) - vec1(1) * vec2(0));
  } else if (vec1.size() == 3) {
    Vec3f v1 = vec1;
    Vec3f v2 = vec2;
    area = 0.5f * (v1.cross(v2)).norm();
  } else {
    throw std::runtime_error("computeArea: vec1.size() != 2 or 3");
  }
  return area;
}

template <int dim>
void ssvd(MatxXf& U, Vecxf& S, MatxXf& V) {
  if (U.determinant() < 0) {
    for (int i = 0; i < dim; i++) {
      U(i, dim - 1) *= -1;
    }
    S(dim - 1) = -S(dim - 1);
  }
  if (V.determinant() < 0) {
    for (int i = 0; i < dim; ++i) {
      V(i, dim - 1) *= -1;
    }
    S(dim - 1) = -S(dim - 1);
  }
}

template void ssvd<2>(MatxXf& U, Vecxf& S, MatxXf& V);
template void ssvd<3>(MatxXf& U, Vecxf& S, MatxXf& V);

Vecxf getMassCenter(const MatxXf& verts, Vecxf& vert_mass) {
  Vecxf center = Vecxf::Zero(verts.cols());
  float total_mass = vert_mass.sum();
  for (int i = 0; i < verts.rows(); ++i) {
    center += verts.row(i) * vert_mass(i);
  }
  center /= total_mass;
  return center;
}

};  // namespace aphys
