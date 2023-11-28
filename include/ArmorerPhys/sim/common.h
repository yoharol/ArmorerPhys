#ifndef ARMORER_SIM_COMMON_H_
#define ARMORER_SIM_COMMON_H_

#include "ArmorerPhys/type.h"
#include <functional>

namespace aphys {

void extract_edge(const Matx3i& faces, Matx2i& edge);

void compute_edge_length(const MatxXd& verts, const Matx2i& edge,
                         Vecxd& length);

void compute_mesh_mass(const MatxXd& verts, const Matx3i& faces,
                       Vecxd& face_mass, Vecxd& vert_mass, double rho = 1.0f);

void generate_gravity_force(const Vecxd& gravity, const Vecxd& vert_mass,
                            MatxXd& gravity_force);

void concatenate_add(Vecxd& A, const MatxXd& B, double scale = 1.0f);
void concatenate_add(MatxXd& A, const Vecxd& B, double scale = 1.0f);
void concatenate_set(Vecxd& A, const MatxXd& B, double scale = 1.0f);
void concatenate_set(MatxXd& A, const Vecxd& B, double scale = 1.0f);
double concatenate_dot(const MatxXd& A, const Vecxd& x, double scale = 1.0f);

struct ImplicitEuler {
  static void predict(MatxXd& predict, const MatxXd& verts, const MatxXd& vel,
                      const MatxXd& external_force, const Vecxd& mass,
                      double dt);
  static double modifyEnergy(double energy, const MatxXd& verts,
                             const MatxXd& p, const Vecxd& mass, double dt);

  // J_g = M / h^2 (x - p) + J
  static void modifyJacobian(Vecxd& J, const MatxXd& verts, const MatxXd& p,
                             const Vecxd& mass, double dt);
  // H_g = M / h^2 I + H
  static void modifyHessian(MatxXd& H, const Vecxd& mass, double dt);
  static void updateVelocity(MatxXd& vel, const MatxXd& verts,
                             const MatxXd& verts_cache, double dt,
                             double damping = 0.0f);
};

// define the energy computation function type
typedef std::function<double(MatxXd&)> EnergyFunc;

// line search along direction dv to minimize energy_func(v+alpha dv)
void line_search(MatxXd& v, MatxXd& v_solver, const Vecxd& dv, const Vecxd& J,
                 EnergyFunc energy_func, double beta = 0.5f,
                 double gamma = 0.03f);

}  // namespace aphys

#endif  // ARMORER_SIM_COMMON_H_