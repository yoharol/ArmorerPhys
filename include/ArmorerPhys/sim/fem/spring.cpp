#include "ArmorerPhys/sim/fem/spring.h"

#include <iostream>

namespace aphys {

float SpringFEM::Energy(MatxXf& verts, const MatxXf& verts_ref,
                        const Matx2i& edges, float k) {
  float energy = 0.0f;
  for (int i = 0; i < edges.rows(); i++) {
    int i1 = edges(i, 0);
    int i2 = edges(i, 1);
    Vecxf p1 = verts.row(i1);
    Vecxf p2 = verts.row(i2);
    Vecxf p1_ref = verts_ref.row(i1);
    Vecxf p2_ref = verts_ref.row(i2);
    float dl = (p1 - p2).norm() - (p1_ref - p2_ref).norm();
    energy += 0.5f * k * dl * dl;
  }
  return energy;
}

// Jacobian of spring energy
void SpringFEM::Jacobian(MatxXf& verts, const MatxXf& verts_ref,
                         const Matx2i& edges, Vecxf& J, float k) {
  int dim = verts.cols();
  J.setZero();
  for (int i = 0; i < edges.rows(); i++) {
    int i1 = edges(i, 0);
    int i2 = edges(i, 1);
    Vecxf p1 = verts.row(i1);
    Vecxf p2 = verts.row(i2);
    Vecxf p1_ref = verts_ref.row(i1);
    Vecxf p2_ref = verts_ref.row(i2);
    float dl = (p1 - p2).norm() - (p1_ref - p2_ref).norm();
    Vecxf dir = (p1 - p2).normalized();
    J.segment(i1 * dim, dim) += k * dl * dir;
    J.segment(i2 * dim, dim) -= k * dl * dir;
  }
}

// Hessian of spring energy
void SpringFEM::Hessian(MatxXf& verts, const MatxXf& verts_ref,
                        const Matx2i& edges, MatxXf& H, float k) {
  int dim = verts.cols();
  // std::cout << edges;
  H.setZero();
  for (int i = 0; i < edges.rows(); i++) {
    int i1 = edges(i, 0);
    int i2 = edges(i, 1);
    Vecxf p1 = verts.row(i1);
    Vecxf p2 = verts.row(i2);
    Vecxf p1_ref = verts_ref.row(i1);
    Vecxf p2_ref = verts_ref.row(i2);
    float l = (p1_ref - p2_ref).norm();
    float pij = (p1 - p2).norm();
    float dl = pij - l;
    Vecxf dir = (p1 - p2).normalized();
    MatxXf h(dim, dim);
    MatxXf I = MatxXf::Identity(dim, dim);
    h = k * I - k * l / pij * (I - dir * dir.transpose());
    H.block(i1 * dim, i1 * dim, dim, dim) += h;
    H.block(i2 * dim, i2 * dim, dim, dim) -= h;
  }
}

}  // namespace aphys