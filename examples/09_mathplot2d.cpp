// free projective dynamics solver

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

int main() {
  GLFWwindow* window = aphys::create_window(
      SCR_WIDTH, SCR_HEIGHT, "Example9: 2D Math Plot with Projective Dynamics");
  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, 0.0f, 1.0f, 0.0f, 1.0f);

  // ===================== create a rectangle =====================
  aphys::MatxXf v_p;
  aphys::Matx3i face_indices;
  aphys::create_rectangle(0.35, 0.65, 20, 0.35, 0.85, 20, v_p, face_indices);
  aphys::Matx2i edge_indices;
  aphys::extract_edge(face_indices, edge_indices);

  // ===================== prepare simulation data =====================
  int substep = 10;
  float dt = 1.0 / 60.0 / (float)substep;
  float devia_stiffness = 10.0f;
  float hydro_stiffness = 10.0f;
  int dim = 2;
  aphys::Vecxf gravity(dim);
  gravity << 0.0f, -3.0f;
  aphys::MatxXf v_vel, v_pred, v_p_ref, v_cache, v_solver;
  aphys::MatxXf v_rig;
  v_p_ref = v_p;
  v_rig.resize(v_p.rows(), v_p.cols());
  v_vel.resize(v_p.rows(), v_p.cols());
  v_vel.setZero();
  v_pred.resize(v_p.rows(), v_p.cols());
  v_solver.resize(v_p.rows(), v_p.cols());
  aphys::Vecxf vert_mass, face_mass;
  aphys::compute_mesh_mass(v_p_ref, face_indices, face_mass, vert_mass);
  aphys::Box2d box(0.0f, 1.0f, 0.0f, 1.0f);
  aphys::Vecxf J(v_p.rows() * dim);
  aphys::MatxXf H(v_p.rows() * dim, v_p.rows() * dim);
  aphys::Vecxf dv(v_p.rows() * dim);
  aphys::MatxXf external_force;
  aphys::generate_gravity_force(gravity, vert_mass, external_force);

  // ================ prepare projective dynamics solver =====================
  aphys::ProjectiveDynamicsSolver<2> pd_solver(
      v_p, v_p_ref, face_indices, face_mass, vert_mass, external_force, dt,
      hydro_stiffness, devia_stiffness);

  // ===================== prepare render =====================
  aphys::Points points = aphys::create_points();
  aphys::set_points_data(points, v_p, aphys::MatxXf());
  points.color = aphys::RGB(255, 0, 0);
  points.point_size = 2.0f;
  aphys::Edges edges = aphys::create_edges();
  aphys::set_edges_data(edges, v_p, edge_indices, aphys::MatxXf());
  edges.color = aphys::RGB(0, 0, 0);
  edges.width = 1.0f;
  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));

  // ===================== prepare math plot =====================
  aphys::Lines grids = aphys::create_grid_axis2d(0.0f, 1.0f, 0.0f, 1.0f, 20, 20,
                                                 aphys::RGB(0, 67, 198));
  grids.alpha = 0.2f;
  aphys::Lines axis =
      aphys::create_axis2d(0.0f, 1.0f, 0.0f, 1.0f, aphys::RGB(0, 21, 98));
  axis.width = 2.f;

  aphys::add_render_func(scene, aphys::get_render_func(axis));
  aphys::add_render_func(scene, aphys::get_render_func(grids));

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    // ===================== simulation =====================
    v_cache = v_p;
    aphys::ImplicitEuler::predict(v_pred, v_p, v_vel, external_force, vert_mass,
                                  dt);
    for (int i = 0; i < 10; i++) {
      pd_solver.localStep(v_p, face_indices);
      pd_solver.globalStep(v_p, v_pred);
      aphys::collision2d(box, v_p);
    }
    aphys::ImplicitEuler::updateVelocity(v_vel, v_p, v_cache, dt);

    aphys::set_points_data(points, v_p, aphys::MatxXf());
    aphys::set_edges_data(edges, v_p, edge_indices, aphys::MatxXf());

    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}