#ifndef ARMORER_SIM_COMMON_H_
#define ARMORER_SIM_COMMON_H_

#include "ArmorerPhys/type.h"
#include <functional>

namespace aphys {

void extract_edge(const Matx3i& faces, Matx2i& edge);

void compute_edge_length(const MatxXf& verts, const Matx2i& edge,
                         Vecxf& length);

void compute_mesh_mass(const MatxXf& verts, const Matx3i& faces,
                       Vecxf& face_mass, Vecxf& vert_mass, float rho = 1.0f);

void generate_gravity_force(const Vecxf& gravity, const Vecxf& vert_mass,
                            MatxXf& gravity_force);

void concatenate_add(Vecxf& A, const MatxXf& B, float scale = 1.0f);
void concatenate_add(MatxXf& A, const Vecxf& B, float scale = 1.0f);
void concatenate_set(Vecxf& A, const MatxXf& B, float scale = 1.0f);
void concatenate_set(MatxXf& A, const Vecxf& B, float scale = 1.0f);
float concatenate_dot(const MatxXf& A, const Vecxf& x, float scale = 1.0f);

struct ImplicitEuler {
  static void predict(MatxXf& predict, const MatxXf& verts, const MatxXf& vel,
                      const MatxXf& external_force, const Vecxf& mass,
                      float dt);
  static float modifyEnergy(float energy, const MatxXf& verts, const MatxXf& p,
                            const Vecxf& mass, float dt);

  // J_g = M / h^2 (x - p) + J
  static void modifyJacobian(Vecxf& J, const MatxXf& verts, const MatxXf& p,
                             const Vecxf& mass, float dt);
  // H_g = M / h^2 I + H
  static void modifyHessian(MatxXf& H, const Vecxf& mass, float dt);
  static void updateVelocity(MatxXf& vel, const MatxXf& verts,
                             const MatxXf& verts_cache, float dt,
                             float damping = 0.0f);
};

// define the energy computation function type
typedef std::function<float(MatxXf&)> EnergyFunc;

// line search along direction dv to minimize energy_func(v+alpha dv)
void line_search(MatxXf& v, MatxXf& v_solver, const Vecxf& dv, const Vecxf& J,
                 EnergyFunc energy_func, float beta = 0.5f,
                 float gamma = 0.03f);

}  // namespace aphys

#endif  // ARMORER_SIM_COMMON_H_