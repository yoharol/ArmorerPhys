#include "ArmorerPhys/sim/common.h"

#include <vector>
#include <algorithm>
#include <iostream>

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/math.h"

namespace aphys {

void compute_edge_length(const MatxXf& verts, const Matx2i& edge,
                         Vecxf& length) {
  int n_edges = edge.rows();
  length.resize(n_edges);
  for (int i = 0; i < n_edges; i++) {
    int i1 = edge(i, 0);
    int i2 = edge(i, 1);
    Vecxf p1 = verts.row(i1);
    Vecxf p2 = verts.row(i2);
    length(i) = (p1 - p2).norm();
  }
}

void extract_edge(const Matx3i& faces, Matx2i& edge) {
  int n_faces = faces.rows();
  std::vector<Vec3i> raw_edges(n_faces * 3);
  for (int i = 0; i < n_faces; i++) {
    for (int k1 = 0; k1 < 3; k1++) {
      int k2 = (k1 + 1) % 3;
      int i1 = faces(i, k1);
      int i2 = faces(i, k2);
      raw_edges[i * 3 + k1] = Vec3i(std::min(i1, i2), std::max(i1, i2), i);
    }
  }
  std::sort(raw_edges.begin(), raw_edges.end(),
            [](const Vec3i& a, const Vec3i& b) {
              if (a(0) < b(0)) return true;
              if (a(0) > b(0)) return false;
              if (a(1) < b(1)) return true;
              return false;
            });

  int i = 0;
  std::vector<Vec2i> raw_edge_indices;
  auto compare = [](Vec3i e1, Vec3i e2) {
    if (e1(0) == e2(0) && e1(1) == e2(1)) return true;
    return false;
  };
  while (i < raw_edges.size()) {
    raw_edge_indices.emplace_back(Vec2i(raw_edges[i](0), raw_edges[i](1)));
    if (i + 1 < raw_edges.size() && compare(raw_edges[i], raw_edges[i + 1]))
      i += 2;
    else
      i += 1;
  }
  edge.resize(raw_edge_indices.size(), 2);
  for (int i = 0; i < raw_edge_indices.size(); i++) {
    edge.row(i) = raw_edge_indices[i];
  }
}

void compute_mesh_mass(const MatxXf& verts, const Matx3i& faces,
                       Vecxf& face_mass, Vecxf& vert_mass) {
  int n_faces = faces.rows();
  int n_verts = verts.rows();
  face_mass.resize(n_faces);
  Vecxf test;
  test.resize(n_verts);
  vert_mass.resize(n_verts);
  vert_mass.setZero();
  for (int i = 0; i < n_faces; i++) {
    int i1 = faces(i, 0);
    int i2 = faces(i, 1);
    int i3 = faces(i, 2);
    Vecxf v1 = verts.row(i1);
    Vecxf v2 = verts.row(i2) - v1.transpose();
    Vecxf v3 = verts.row(i3) - v1.transpose();
    float area = computeArea(v2, v3);
    face_mass(i) = area;
    vert_mass(i1) += area / 3.0f;
    vert_mass(i2) += area / 3.0f;
    vert_mass(i3) += area / 3.0f;
  }
}

void generate_gravity_force(const Vecxf& gravity, const Vecxf& vert_mass,
                            MatxXf& gravity_force) {
  int n_verts = vert_mass.size();
  int dim = gravity.size();
  gravity_force.resize(n_verts, dim);
  for (int i = 0; i < n_verts; i++) {
    gravity_force.row(i) = vert_mass(i) * gravity.transpose();
  }
}

void concatenate_add(Vecxf& A, const MatxXf& B, float scale) {
  assert(A.size() == B.rows() * B.cols());
  int dim = B.cols();
  for (int i = 0; i < B.rows(); i++) {
    A.segment(i, dim) += scale * B.row(i).transpose();
  }
}
void concatenate_add(MatxXf& A, const Vecxf& B, float scale) {
  assert(B.size() == A.rows() * A.cols());
  int dim = A.cols();
  for (int i = 0; i < A.rows(); i++) {
    A.row(i) += scale * B.segment(i * dim, dim).transpose();
  }
}

void concatenate_set(Vecxf& A, const MatxXf& B, float scale) {
  assert(A.size() == B.rows() * B.cols());
  int dim = B.cols();
  for (int i = 0; i < B.rows(); i++) {
    A.segment(i, dim) = scale * B.row(i).transpose();
  }
}

void concatenate_set(MatxXf& A, const Vecxf& B, float scale) {
  assert(B.size() == A.rows() * A.cols());
  int dim = A.cols();
  for (int i = 0; i < A.rows(); i++) {
    A.row(i) = scale * B.segment(i * dim, dim).transpose();
  }
}

float concatenate_dot(const MatxXf& A, const Vecxf& x, float scale) {
  assert(x.size() == A.rows() * A.cols());
  int dim = A.cols();
  float result = 0.0f;
  for (int i = 0; i < A.rows(); i++) {
    result += scale * A.row(i).dot(x.segment(i * dim, dim));
  }
  return result;
}

void ImplicitEuler::predict(MatxXf& predict, const MatxXf& verts,
                            const MatxXf& vel, const MatxXf& external_force,
                            const Vecxf& mass, float dt) {
  int n_verts = verts.rows();
  int dim = verts.cols();
  for (int i = 0; i < n_verts; i++) {
    predict.row(i) = verts.row(i) + dt * vel.row(i) +
                     dt * dt / mass(i) * external_force.row(i);
  }
}

float ImplicitEuler::modifyEnergy(float energy, const MatxXf& verts,
                                  const MatxXf& p, const Vecxf& mass,
                                  float dt) {
  int n_verts = verts.rows();
  for (int i = 0; i < n_verts; i++) {
    energy +=
        mass(i) / 2.0f / dt / dt * (verts.row(i) - p.row(i)).squaredNorm();
  }
  return energy;
}

// J_g = M / h^2 (x - p) + J
void ImplicitEuler::modifyJacobian(Vecxf& J, const MatxXf& verts,
                                   const MatxXf& p, const Vecxf& mass,
                                   float dt) {
  int n_verts = verts.rows();
  int dim = verts.cols();
  for (int i = 0; i < n_verts; i++) {
    J.segment(i * dim, dim) += mass(i) / dt / dt * (verts.row(i) - p.row(i));
  }
}

// H_g = M / h^2 I + H
void ImplicitEuler::modifyHessian(MatxXf& H, const Vecxf& mass, float dt) {
  int n_verts = mass.size();
  int dim = H.cols() / n_verts;
  for (int i = 0; i < n_verts; i++) {
    H.block(i * dim, i * dim, dim, dim) +=
        mass(i) / dt / dt * MatxXf::Identity(dim, dim);
  }
}

void ImplicitEuler::updateVelocity(MatxXf& vel, const MatxXf& verts,
                                   const MatxXf& verts_cache, float dt,
                                   float damping) {
  int n_verts = verts.rows();
  for (int i = 0; i < n_verts; i++) {
    vel.row(i) = (verts.row(i) - verts_cache.row(i)) / dt * exp(-damping * dt);
  }
}

// line search along direction dv to minimize energy_func(v+alpha dv)
void line_search(MatxXf& v, MatxXf& v_solver, const Vecxf& dv, const Vecxf& J,
                 EnergyFunc energy_func, float beta, float gamma) {
  float alpha = 1.0f;
  v_solver = v;
  concatenate_add(v_solver, alpha * dv);
  float ddv = dv.transpose() * J;
  int iter = 0;
  while (energy_func(v_solver) > energy_func(v) + gamma * alpha * ddv) {
    alpha *= beta;
    v_solver = v;
    concatenate_add(v_solver, alpha * dv);
    if (iter++ > 100) {
      std::cout << "line search failed" << std::endl;
      break;
    }
  }
  v = v_solver;
}

}  // namespace aphys
