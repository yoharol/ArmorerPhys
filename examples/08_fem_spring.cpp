#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/fem.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window = aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example8: FEM spring simulation");

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, 0.0f, 1.0f, 0.0f, 1.0f);

  // ===================== create a rectangle =====================
  aphys::MatxXd v_p;
  aphys::Matx3i face_indices;
  aphys::create_rectangle(0.35, 0.65, 10, 0.55, 0.85, 10, v_p, face_indices);
  aphys::Matx2i edge_indices;
  aphys::extract_edge(face_indices, edge_indices);

  // ===================== prepare simulation data =====================
  int substep = 20;
  double dt = 1.0 / 60.0 / (double)substep;
  double stiffness = 40.0f;
  int dim = 2;
  aphys::Vecxd gravity(dim);
  gravity << 0.0, -3.0;
  aphys::MatxXd v_vel, v_pred, v_p_ref, v_cache, v_solver;
  v_p_ref = v_p;
  v_vel.resize(v_p.rows(), v_p.cols());
  v_vel.setZero();
  v_pred.resize(v_p.rows(), v_p.cols());
  v_solver.resize(v_p.rows(), v_p.cols());
  aphys::Vecxd vert_mass, face_mass;
  aphys::compute_mesh_mass(v_p_ref, face_indices, face_mass, vert_mass);
  aphys::Box2d box(0.0, 1.0, 0.0, 1.0);
  aphys::Vecxd J(v_p.rows() * dim);
  aphys::MatxXd H(v_p.rows() * dim, v_p.rows() * dim);
  aphys::Vecxd dv(v_p.rows() * dim);
  aphys::MatxXd external_force;
  aphys::generate_gravity_force(gravity, vert_mass, external_force);

  // ===================== prepare render =====================
  aphys::Points points = aphys::create_points();
  aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
  points.color = aphys::RGB(255, 0, 0);
  points.point_size = 2.0f;
  aphys::Edges edges = aphys::create_edges();
  aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());
  edges.color = aphys::RGB(0, 0, 0);
  edges.width = 1.0f;
  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    // implicit euler is to solve the following equation:
    // p = x + v * dt + f_ext * dt^2 / m
    // M/h^2 (x - p) + J(x) = 0
    // With newton method, we have:
    // dx = (M/h^2 + H(x))^-1 (M/h^2 (x-p) + J(x))
    for (int _ = 0; _ < substep; _++) {
      v_cache = v_p;
      aphys::ImplicitEuler::predict(v_pred, v_p, v_vel, external_force,
                                    vert_mass, dt);
      int solver_step = 0;
      while (true) {
        aphys::SpringFEM::Jacobian(v_p, v_p_ref, edge_indices, J, stiffness);
        aphys::ImplicitEuler::modifyJacobian(J, v_p, v_pred, vert_mass, dt);
        if (J.norm() < 1e-2) break;
        aphys::SpringFEM::Hessian(v_p, v_p_ref, edge_indices, H, stiffness);
        aphys::ImplicitEuler::modifyHessian(H, vert_mass, dt);
        dv = -H.colPivHouseholderQr().solve(J);
        aphys::line_search(v_p, v_solver, dv, J, [&](aphys::MatxXd& v) {
          return aphys::ImplicitEuler::modifyEnergy(
              aphys::SpringFEM::Energy(v, v_p_ref, edge_indices, stiffness), v,
              v_pred, vert_mass, dt);
        });
      }
      // v_p = v_pred;

      aphys::collision2d(box, v_p);
      aphys::ImplicitEuler::updateVelocity(v_vel, v_p, v_cache, dt);
    }

    glfwPollEvents();

    aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
    aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                          aphys::MatxXf());

    aphys::set_background_RGB({244, 244, 244});

    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}