#ifndef ARMORER_SIM_COMMON_H_
#define ARMORER_SIM_COMMON_H_

#include "ArmorerPhys/type.h"
#include <functional>

namespace aphys {

void compute_mesh_mass(const MatxXd& verts, const Matx3i& faces,
                       Vecxd& face_mass, Vecxd& vert_mass, double rho = 1.0f);

void compute_tet_mass(const MatxXd& verts, const Matx4i& tets, Vecxd& tet_mass,
                      Vecxd& vert_mass, double rho = 1.0f);

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

  static void solveDeltaVelocity(MatxXd& delta_vel,                //
                                 const MatxXd& verts,              //
                                 const MatxXd& external_force,     //
                                 const Vecxd& mass,                //
                                 const Vecxd& J, const MatxXd& H,  //
                                 const double dt);
};

// define the energy computation function type
typedef std::function<double(MatxXd&)> EnergyFuncMatBased;
typedef std::function<double(Vecxd&)> EnergyFuncVecBased;

// line search along direction dv to minimize energy_func(v+alpha dv)
void line_search_mat(MatxXd& v, MatxXd& v_solver, const Vecxd& dv,
                     const Vecxd& J, EnergyFuncMatBased energy_func,
                     double beta = 0.5, double gamma = 0.03);
void line_search_vec(Vecxd& v, Vecxd& v_solver, const Vecxd& dv, const Vecxd& J,
                     EnergyFuncVecBased energy_func, double beta = 0.5,
                     double gamma = 0.03);

}  // namespace aphys

#endif  // ARMORER_SIM_COMMON_H_
