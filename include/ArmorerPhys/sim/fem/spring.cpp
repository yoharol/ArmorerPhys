#include "ArmorerPhys/sim/fem/spring.h"

namespace aphys {

double SpringFEM::Energy(MatxXd& verts, const MatxXd& verts_ref,
                         const Matx2i& edges, double k) {
  double energy = 0.0f;
  for (int i = 0; i < edges.rows(); i++) {
    int i1 = edges(i, 0);
    int i2 = edges(i, 1);
    Vecxd p1 = verts.row(i1);
    Vecxd p2 = verts.row(i2);
    Vecxd p1_ref = verts_ref.row(i1);
    Vecxd p2_ref = verts_ref.row(i2);
    double dl = (p1 - p2).norm() - (p1_ref - p2_ref).norm();
    energy += 0.5f * k * dl * dl;
  }
  return energy;
}

// Jacobian of spring energy
void SpringFEM::Jacobian(const MatxXd& verts, const MatxXd& verts_ref,
                         const Matx2i& edges, Vecxd& J, double k) {
  int dim = verts.cols();
  J.setZero();
  for (int i = 0; i < edges.rows(); i++) {
    int i1 = edges(i, 0);
    int i2 = edges(i, 1);
    Vecxd p1 = verts.row(i1);
    Vecxd p2 = verts.row(i2);
    Vecxd p1_ref = verts_ref.row(i1);
    Vecxd p2_ref = verts_ref.row(i2);
    double dl = (p1 - p2).norm() - (p1_ref - p2_ref).norm();
    Vecxd dir = (p1 - p2).normalized();
    J.segment(i1 * dim, dim) += k * dl * dir;
    J.segment(i2 * dim, dim) -= k * dl * dir;
  }
}

// Hessian of spring energy
void SpringFEM::Hessian(const MatxXd& verts, const MatxXd& verts_ref,
                        const Matx2i& edges, MatxXd& H, double k) {
  int dim = verts.cols();
  // std::cout << edges;
  H.setZero();
  for (int i = 0; i < edges.rows(); i++) {
    int i1 = edges(i, 0);
    int i2 = edges(i, 1);
    Vecxd p1 = verts.row(i1);
    Vecxd p2 = verts.row(i2);
    Vecxd p1_ref = verts_ref.row(i1);
    Vecxd p2_ref = verts_ref.row(i2);
    double l = (p1_ref - p2_ref).norm();
    double pij = (p1 - p2).norm();
    double dl = pij - l;
    Vecxd dir = (p1 - p2).normalized();
    MatxXd h(dim, dim);
    MatxXd I = MatxXd::Identity(dim, dim);
    h = k * I - k * l / pij * (I - dir * dir.transpose());
    H.block(i1 * dim, i1 * dim, dim, dim) += h;
    H.block(i2 * dim, i2 * dim, dim, dim) -= h;
  }
}

}  // namespace aphys