#ifndef GL_MATH_H_
#define GL_MATH_H_

#include <random>
#include <stdexcept>

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include "math.h"

#include "ArmorerPhys/type.h"

namespace aphys {

template <typename T>
inline T clamp(T value, T min, T max) {
  return std::max(min, std::min(max, value));
}

template <typename T>
inline T lerp(T a, T b, double t) {
  return (1.0 - t) * a + t * b;
}

inline float random(float rMin, float rMax) {
  const float rRandMax = 1. / (float)RAND_MAX;
  float u = rRandMax * (float)rand();
  return u * (rMax - rMin) + rMin;
}

inline double closest_point_on_segment(const Vecxd& a, const Vecxd& b,
                                       const Vecxd& refp) {
  Vecxd p = b - a;
  double t = clamp((refp - a).dot(p) / p.dot(p), 0., 1.);
  return t;
}

inline double det2D(const Vecxd& a, const Vecxd& b) {
  return a(0) * b(1) - a(1) * b(0);
}

inline bool segment_intersection2D(const Vecxd& a, const Vecxd& b,
                                   const Vecxd& c, const Vecxd& d, double t1,
                                   double t2) {
  Vecxd p = b - a;
  Vecxd q = d - c;
  double det = det2D(p, q);
  if (det == 0) return false;
  Vecxd r = c - a;
  t1 = det2D(r, q) / det;
  t2 = det2D(r, p) / det;
  if (t1 >= 0. && t1 <= 1. && t2 >= 0. && t2 <= 1.) return true;
  return false;
}

struct RandomEngine {
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<double> dis;

  double operator()() { return dis(gen); }
  double operator()(double minv, double maxv) {
    return dis(gen) * (maxv - minv) + minv;
  }

  static RandomEngine& getInstance() {
    static RandomEngine engine;
    return engine;
  }
  RandomEngine(RandomEngine const&) = delete;
  RandomEngine(RandomEngine&&) = delete;

 private:
  RandomEngine() : gen(rd()), dis(0.0, 1.0) {}
};

Vec3d heat_rgb(double value, double minv, double maxv);

template <typename Func>
inline void for_each_nonzero(SparseMatd& matrix, Func func) {
  for (int k = 0; k < matrix.outerSize(); ++k) {
    for (SparseMatd::InnerIterator it(matrix, k); it; ++it) {
      func(it);
    }
  }
}

template <typename T>
T sum_vector(const std::vector<T>& vec) {
  T sum = 0;
  for (const auto& v : vec) {
    sum += v;
  }
  return sum;
}

void set_sparse_block(SparseMatd& mat, const MatxXd& block, int row, int col,
                      int n, int m);

void set_sparse_block_from_diagnol(SparseMatd& mat, const Vecxd& diag, int row,
                                   int col);

void set_diag_matrix(const Vecxd& diag_vec, DiagMatxXd& diag_mat, int expand);

void Mat3fToList3f(const Matx3d& mat, Listx3f& vec);

void List3fToMat3f(const Listx3f& vec, Matx3d& mat);

void List3fToList2f(const Listx3f& vec3, Listx2f& vec2);

void List2fToList3f(const Listx2f& vec2, Listx3f& vec3);

void Mat2fToList3f(const Matx2d& mat, Listx3f& vec);

double computeArea(Vecxd vec1, Vecxd vec2);

double computeVolume(const Vec3d vec1, const Vec3d vec2, const Vec3d vec3);

template <int dim>
void ssvd(MatxXd& U, Vecxd& S, MatxXd& V);

template <int dim>
void SVD(const MatxXd& A, MatxXd& U, Vecxd& S, MatxXd& V);

template <int dim>
MatxXd rotation_extraction(const MatxXd& A);

template <int dim>
void polar_decomposition(const MatxXd& A, MatxXd& R, MatxXd& S);

Vecxd getMassCenter(const MatxXd& verts, Vecxd& vert_mass);

}  // namespace aphys

#endif  // GL_MATH_H_
