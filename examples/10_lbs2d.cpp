#include <iostream>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>

#include "ArmorerPhys/SimCore.h"

int main() {
  Eigen::MatrixXf verts, verts_pred, verts_solved;
  Eigen::SparseLU<aphys::SparseMatf> solver;
  aphys::SparseMatf M_h2;

  M_h2.resize(4, 4);
  verts.resize(4, 2);
  verts_pred.resize(4, 2);

  for (int i = 0; i < 4; i++) {
    verts(i, 0) = i;
    verts(i, 1) = i + 1;
    verts_pred(i, 0) = i;
    verts_pred(i, 1) = i + 1;
    M_h2.insert(i, i) = aphys::RandomEngine::getInstance()();
  }

  solver.analyzePattern(M_h2);
  solver.factorize(M_h2);

  for (int _ = 0; _ < 100; _++) {
    Eigen::MatrixXf rhs;
    rhs = M_h2 * verts_pred;
    verts_solved = solver.solve(rhs);
    verts_pred = verts_solved;
  }

  std::cout << verts_solved << std::endl;
}