// Projective dynamics with interactive gui with imgui

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window = aphys::create_window(
      SCR_WIDTH, SCR_HEIGHT, "Example7: interactive projective dynamics");
  double bottom = 0.0f;
  double top = 1.0f;
  double left = 0.0f;
  double right = 1.0f;

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, left, right, bottom, top);
  aphys::Gui gui = aphys::create_gui(window, "gui");
  gui.width = SCR_WIDTH;
  gui.height = 80;

  // ===================== create a rectangle =====================
  aphys::MatxXd v_p;
  aphys::Matx3i face_indices;
  aphys::create_rectangle(0.35, 0.65, 20, 0.35, 0.85, 20, v_p, face_indices);
  aphys::Matx2i edge_indices;
  aphys::extract_edge(face_indices, edge_indices);

  // ===================== prepare simulation data =====================
  int substep = 1;
  double dt = 1.0f / 100.0f;
  double devia_stiffness = 1.0f;
  double hydro_stiffness = 1.0f;
  double rho = 1000.0f;
  int dim = 2;
  aphys::Vecxd gravity(dim);
  gravity << 0.0f, -3.0f;
  aphys::MatxXd v_vel, v_pred, v_p_ref, v_cache, v_solver;
  aphys::MatxXd v_rig;
  v_p_ref = v_p;
  v_rig.resize(v_p.rows(), v_p.cols());
  v_vel.resize(v_p.rows(), v_p.cols());
  v_vel.setZero();
  v_pred.resize(v_p.rows(), v_p.cols());
  v_solver.resize(v_p.rows(), v_p.cols());
  aphys::Vecxd vert_mass, face_mass;
  aphys::compute_mesh_mass(v_p_ref, face_indices, face_mass, vert_mass, rho);
  aphys::Box2d box(0.0f, 1.0f, 0.0f, 1.0f);
  aphys::Vecxd J(v_p.rows() * dim);
  aphys::MatxXd H(v_p.rows() * dim, v_p.rows() * dim);
  aphys::Vecxd dv(v_p.rows() * dim);
  aphys::MatxXd external_force;
  aphys::generate_gravity_force(gravity, vert_mass, external_force);

  // ===================== prepare control point =====================
  aphys::MatxXd v_control(2, 2);
  aphys::MatxXd v_control_ref, control_vector;
  v_control << 0.5, 0.35, 0.5, 0.85;
  v_control_ref = v_control;
  control_vector.resize(v_control.rows(), v_control.cols());
  aphys::MatxXd v_control_weights(2, v_p.rows());
  aphys::MatxXd v_comp_weights(2, v_p.rows());
  double mass_sum = vert_mass.sum();
  for (int i = 0; i < v_p.rows(); i++) {
    double y = v_p(i, 1);
    double t = (y - 0.35f) / (0.85f - 0.35f);
    v_control_weights(0, i) = 1 - t;
    v_control_weights(1, i) = t;
    v_comp_weights(0, i) = (1 - t) * vert_mass(i);
    v_comp_weights(1, i) = t * vert_mass(i);
  }
  aphys::MatxXd lambda(2, 2);

  // ================ prepare projective dynamics solver =====================
  aphys::ProjectiveDynamicsSolver<2> pd_solver(
      v_p, v_p_ref, face_indices, face_mass, vert_mass, external_force, dt,
      hydro_stiffness, devia_stiffness);
  aphys::ControlledProjDynSolver cpd_solver(&pd_solver, v_comp_weights);

  // ===================== prepare render =====================
  aphys::Points control_point = aphys::create_points();
  aphys::set_points_data(control_point, v_control.cast<float>(),
                         aphys::MatxXf());
  control_point.color = aphys::RGB(0, 0, 255);
  control_point.point_size = 10.0f;
  aphys::Points points = aphys::create_points();
  aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
  points.color = aphys::RGB(255, 0, 0);
  points.point_size = 2.0f;
  aphys::Edges edges = aphys::create_edges();
  aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());
  edges.color = aphys::RGB(0, 0, 0);
  edges.width = 1.0f;
  aphys::Edges rig_edges = aphys::create_edges();
  aphys::set_edges_data(rig_edges, v_rig.cast<float>(), edge_indices,
                        aphys::MatxXf());
  rig_edges.color = aphys::RGB(0, 0, 255);
  rig_edges.alpha = 0.1f;
  rig_edges.width = 1.0f;
  aphys::add_render_func(scene, aphys::get_render_func(control_point));
  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));
  aphys::add_render_func(scene, aphys::get_render_func(rig_edges));

  auto lbs = [&]() {
    for (int i = 0; i < v_p.rows(); i++) {
      v_rig.row(i) = aphys::Vec2d(0.0f, 0.0f);
      for (int j = 0; j < v_control.rows(); j++) {
        v_rig.row(i) +=
            v_control_weights(j, i) *
            (v_control.row(j) - v_control_ref.row(j) + v_p_ref.row(i));
      }
    }
    control_vector = v_comp_weights * v_rig;
  };

  // ===================== prepare gui function =====================
  aphys::Vecxd lambda_norm(480);
  aphys::Vecxf lambda_norm_f(480);
  lambda_norm.setZero();
  bool dragging = false;

  aphys::add_gui_func(gui, [&]() {
    lambda_norm_f = lambda_norm.cast<float>();
    ImGui::PlotLines("lambda", lambda_norm_f.data(), lambda_norm_f.size(), 0,
                     nullptr, 0.0f, 3000.0f, ImVec2(SCR_WIDTH, 50));
  });

  aphys::add_gui_mouse_input_func(gui, [&]() {
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      ImVec2 pos = ImGui::GetMousePos();
      float x = pos.x / SCR_WIDTH;
      float y = 1.0 - pos.y / SCR_HEIGHT;
      aphys::camera2d_screen_to_world(scene.camera, x, y);

      aphys::Vecxd v = v_control.row(0);
      aphys::Vecxd target(2);
      target << x, y;
      double interpolate = 1.0f;
      if ((v - target).norm() < 0.01f) {
        v_control.row(0) = target;
      } else {
        v_control.row(0) = (1 - interpolate) * v + interpolate * target;
      }
      dragging = true;
    }
    /*if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
      dragging = false;
    }
    if (dragging) {
      ImVec2 pos = ImGui::GetMousePos();
      double x = pos.x / SCR_WIDTH;
      double y = 1.0 - pos.y / SCR_HEIGHT;
      aphys::camera2d_screen_to_world(scene.camera, x, y);
      v_control.row(0) = aphys::Vec2d(x, y);
    }*/
  });

  glfwSwapInterval(1);
  int frame = 0;
  while (!glfwWindowShouldClose(window)) {
    // projective dynamcis solver
    // within the implicit euler framework
    // for (int _ = 0; _ < substep; _++) {
    v_cache = v_p;
    aphys::ImplicitEuler::predict(v_pred, v_p, v_vel, external_force, vert_mass,
                                  dt);
    lbs();
    for (int i = 0; i < 100; i++) {
      pd_solver.localStep(v_p, face_indices);
      cpd_solver.globalStep(v_solver, v_pred, control_vector, lambda);
      double error = (v_solver - v_p).norm();
      v_p = v_solver;
      if (error < 1e-5) break;
    }

    aphys::ImplicitEuler::updateVelocity(v_vel, v_p, v_cache, dt);
    // }

    aphys::set_points_data(control_point, v_control.cast<float>(),
                           aphys::MatxXf());
    aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
    aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                          aphys::MatxXf());
    aphys::set_edges_data(rig_edges, v_rig.cast<float>(), edge_indices,
                          aphys::MatxXf());
    lambda_norm((frame++) % 480) = lambda.norm();
    glfwPollEvents();
    aphys::handle_gui_input(gui);

    aphys::set_background_RGB({244, 244, 244});

    aphys::render_scene(scene);
    aphys::render_gui(gui);

    glfwSwapBuffers(window);
  }
  aphys::destroy_gui(gui);
  glfwTerminate();
}