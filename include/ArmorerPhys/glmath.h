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

template <typename T>
inline T clamp(T value, T min, T max) {
  return std::max(min, std::min(max, value));
}

Vec3d heat_rgb(double value, double minv, double maxv);

void set_sparse_block(SparseMatd& mat, const MatxXd& block, int row, int col,
                      int n, int m);

void set_diag_matrix(Vecxd& diag_vec, DiagMatxXd& diag_mat, int expand);

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
