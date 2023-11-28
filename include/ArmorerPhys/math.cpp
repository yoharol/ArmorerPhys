#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <cassert>
#include <cmath>

#include "ArmorerPhys/math.h"

namespace aphys {

Vec3d heat_rgb(double value, double minv, double maxv) {
  double ratio = 2 * (value - minv) / (maxv - minv);
  double b = std::max(0.0, 1 - ratio);
  double r = std::max(0.0, ratio - 1);
  double g = 1 - b - r;
  return Vec3d(r, g, b);
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

double computeArea(const Vecxd& vec1, const Vecxd& vec2) {
  if (vec1.size() != vec2.size()) {
    throw std::runtime_error("computeArea: vec1.size() != vec2.size()");
  }
  double area = 0.0;
  if (vec1.size() == 2) {
    area = 0.5 * (vec1(0) * vec2(1) - vec1(1) * vec2(0));
  } else if (vec1.size() == 3) {
    Vec3d v1 = vec1;
    Vec3d v2 = vec2;
    area = 0.5 * (v1.cross(v2)).norm();
  } else {
    throw std::runtime_error("computeArea: vec1.size() != 2 or 3");
  }
  return area;
}

template <int dim>
void ssvd(MatxXd& U, Vecxd& S, MatxXd& V) {
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

template void ssvd<2>(MatxXd& U, Vecxd& S, MatxXd& V);
template void ssvd<3>(MatxXd& U, Vecxd& S, MatxXd& V);

Vecxd getMassCenter(const MatxXd& verts, Vecxd& vert_mass) {
  Vecxd center = Vecxd::Zero(verts.cols());
  double total_mass = vert_mass.sum();
  for (int i = 0; i < verts.rows(); ++i) {
    center += verts.row(i) * vert_mass(i);
  }
  center /= total_mass;
  return center;
}

};  // namespace aphys
