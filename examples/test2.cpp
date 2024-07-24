// bounded biharmonic weights

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/fem.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window = aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example21: arap_interpolation");
  double bottom = -3.0f;
  double top = 3.0f;
  double left = -3.0f;
  double right = 3.0f;

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, left, right, bottom, top);

  aphys::Gui gui = aphys::create_gui(window, "gui");
  gui.width = 350;
  gui.height = 140;

  // ================= create the outside edge of a capsule ==================
  aphys::MatxXd v_p, v_p_ref;
  aphys::MatxXd v_donut;
  aphys::MatxXd v_twist1;
  aphys::Matx3i face_indices;
  aphys::Matx2i edge_indices;

  aphys::readOBJ(std::string(ASSETS_PATH) + "/rect_twist/ref.obj", v_p_ref,
                 face_indices, 2);
  aphys::readOBJ(std::string(ASSETS_PATH) + "/rect_twist/donut.obj", v_donut,
                 face_indices, 2);
  aphys::readOBJ(std::string(ASSETS_PATH) + "/rect_twist/twist2.obj", v_twist1,
                 face_indices, 2);

  v_p = v_p_ref;
  aphys::extract_edge(face_indices, edge_indices);

  double weight = 0.0;

  // ===================== prepare simulation ======================
  aphys::Vecxd face_mass, vert_mass;
  aphys::MatxXd external_force;
  aphys::compute_mesh_mass(v_p_ref, face_indices, face_mass, vert_mass, 1000.0);
  aphys::Vecxd gravity(2);
  gravity << 0.0, 0.0;
  aphys::generate_gravity_force(gravity, vert_mass, external_force);
  aphys::MatxXd source_B;
  aphys::NeoHookeanFEM2D::project_B(v_p, v_p_ref, face_indices, source_B);
  // aphys::ARAPTargetShape2D idle_shape(v_p, face_indices, source_B);
  aphys::ARAPTargetShape2D donut_shape(v_donut, face_indices, source_B);
  aphys::ARAPTargetShape2D twist1_shape(v_twist1, face_indices, source_B);

  for (int i = 100; i < face_indices.rows(); i++) {
    if (donut_shape.rotate_angle(i) < 0.0)
      donut_shape.rotate_angle(i) += 2.0 * M_PI;
  }

  aphys::MatxXd target_F = source_B;
  aphys::Vecxi fixed_verts(1);
  fixed_verts << 0;
  int iter_number = 0;
  double pow_num = 1.0;
  aphys::MatxXd fixed_pos(1, 2);
  fixed_pos.row(0) = v_p_ref.row(0);
  aphys::ARAPInterpolate2D arap_solver(v_p, v_p_ref, face_indices, face_mass,
                                       vert_mass, external_force, 0.0, 1.0,
                                       fixed_verts);
  aphys::MatxXd I(2, face_indices.rows() * 2);
  for (int i = 0; i < face_indices.rows(); i++)
    I.block(0, i * 2, 2, 2) = aphys::MatxXd::Identity(2, 2);
  aphys::MatxXd interpolated_S(2, face_indices.rows() * 2);

  auto arap_interpolate = [&]() {
    weight = std::pow(weight, pow_num);
    interpolated_S = weight * twist1_shape.S + (1.0 - weight) * donut_shape.S;
    // aphys::MatxXd its1 = weight * twist1_shape.S + (1.0 - weight) * I;
    // aphys::MatxXd its2 = weight * I + (1.0 - weight) * donut_shape.S;
    // interpolated_S = weight * its1 + (1.0 - weight) * its2;

    aphys::Vecxd interpolate_rotate = weight * twist1_shape.rotate_angle +
                                      (1.0 - weight) * donut_shape.rotate_angle;
    aphys::ARAPTargetShape2D::recover_deformation_map(
        interpolated_S, interpolate_rotate, target_F);
    arap_solver.solver_static_shape(v_p, face_indices, fixed_pos, target_F);
    aphys::MatxXd v_cache = v_p;
    for (int _ = 0; _ < iter_number; _++) {
      arap_solver.local_step(v_cache, face_indices, target_F);
      arap_solver.solver_static_shape(v_cache, face_indices, fixed_pos,
                                      target_F);
    }
    double t = 4.0 * (0.5 - weight) * (0.5 - weight);
    aphys::MatxXd ending = t * v_p + (1.0 - t) * v_cache;
    v_p = ending;
  };

  auto linear_interpolate = [&](aphys::Vec3d bary) {
    v_p = weight * v_twist1 + (1.0 - weight) * v_donut;
  };

  // ===================== prepare render data =====================
  aphys::Points points = aphys::create_points();
  points.point_size = 3.0f;

  aphys::Edges edges = aphys::create_edges();
  edges.color = aphys::RGB(0, 0, 0);
  edges.width = 1.0f;

  aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
  aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());

  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));

  float weightSlider = 0.0;
  int iter_input = 0;
  float pownum_input = 1.0f;

  aphys::Vecxd U(100);
  double max_U = 0.0;

  auto compute_energy_line = [&]() {
    max_U = 0.0;
    for (int i = 0; i <= 100; i++) {
      double t = i / 100.0;
      weight = t;
      arap_interpolate();
      U(i) = 0.5 * (interpolated_S - I).squaredNorm();
      max_U = std::max(max_U, U(i));
    }
  };
  compute_energy_line();

  aphys::add_gui_func(gui, [&]() {
    aphys::Vecxf U_f = U.cast<float>();
    ImGui::InputInt("iters", &iter_input);
    ImGui::SliderFloat("weight", &weightSlider, 0.0, 1.0, "%.3f");
    ImGui::PlotLines("Energy", U_f.data(), U_f.size(), 0, nullptr, 0.0f, max_U,
                     ImVec2(300, 50));
    if (weight != weightSlider || iter_number != iter_input) {
      weight = weightSlider;
      iter_number = iter_input;
      arap_interpolate();
      aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
      aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                            aphys::MatxXf());
    }
    ImGui::InputFloat("Pow", &pownum_input);
    if (pownum_input != pow_num) {
      pow_num = pownum_input;
      compute_energy_line();
    }
  });

  // ===================== main loop =====================
  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    aphys::handle_gui_input(gui);
    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);
    aphys::render_gui(gui);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
