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
  gui.width = 300;
  gui.height = 50;

  // ================= create the outside edge of a capsule ==================
  aphys::MatxXd v_p, v_p_ref;
  aphys::MatxXd v_donut;
  aphys::MatxXd v_twist1;
  aphys::MatxXd v_twist2;
  aphys::Matx3i face_indices;
  aphys::Matx2i edge_indices;

  aphys::readOBJ(std::string(ASSETS_PATH) + "/rect_twist/ref.obj", v_p_ref,
                 face_indices, 2);
  aphys::readOBJ(std::string(ASSETS_PATH) + "/rect_twist/donut.obj", v_donut,
                 face_indices, 2);
  aphys::readOBJ(std::string(ASSETS_PATH) + "/rect_twist/twist1.obj", v_twist1,
                 face_indices, 2);
  aphys::readOBJ(std::string(ASSETS_PATH) + "/rect_twist/twist2.obj", v_twist2,
                 face_indices, 2);
  // v_twist2 = v_p_ref;
  v_p = v_p_ref;

  aphys::extract_edge(face_indices, edge_indices);

  aphys::MatxXd interpolate_triangle(3, 2);
  interpolate_triangle << -2.5, -2.0, 2.5, -2.0, 0.0, 2.33;
  aphys::Matx3i interpolate_face(1, 3);
  interpolate_face << 0, 1, 2;
  aphys::Matx2i interpolate_edges(3, 2);
  interpolate_edges << 0, 1, 1, 2, 2, 0;

  aphys::MatxXd weight_pos(1, 2);

  auto set_interpolation = [&](aphys::Vec2d pos) -> aphys::Vec3d {
    // aphys::Vec2d p = weight_pos.row(0);
    aphys::Vec2d p1 = interpolate_triangle.row(0);
    aphys::Vec2d p2 = interpolate_triangle.row(1);
    aphys::Vec2d p3 = interpolate_triangle.row(2);
    // limit pos in the area of triangle p1, p2, p3

    aphys::Vec3d bary;
    aphys::compute_barycentric_triangle(pos, p1, p2, p3, bary);
    weight_pos.row(0) =
        (bary(0) * p1 + bary(1) * p2 + bary(2) * p3).transpose();
    return bary;
  };

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
  aphys::ARAPTargetShape2D twist2_shape(v_twist2, face_indices, source_B);

  for (int i = 100; i < face_indices.rows(); i++) {
    if (donut_shape.rotate_angle(i) < 0.0)
      donut_shape.rotate_angle(i) += 2.0 * M_PI;
  }
  aphys::MatxXd target_F = source_B;
  aphys::Vecxi fixed_verts(1);
  fixed_verts << 0;
  aphys::MatxXd fixed_pos(1, 2);
  fixed_pos.row(0) = v_p_ref.row(0);
  aphys::ARAPInterpolate2D arap_solver(v_p, v_p_ref, face_indices, face_mass,
                                       vert_mass, external_force, 1.0, 0.0,
                                       fixed_verts);

  auto raw_arap_interpolate = [&](aphys::Vec3d bary) {
    aphys::MatxXd interpolated_F = bary(0) * twist1_shape.F +
                                   bary(1) * twist2_shape.F +
                                   bary(2) * donut_shape.F;
    arap_solver.solver_static_shape(v_p, face_indices, fixed_pos,
                                    interpolated_F);
  };

  auto arap_interpolate = [&](aphys::Vec3d bary) {
    aphys::MatxXd interpolated_S = bary(0) * twist1_shape.S +
                                   bary(1) * twist2_shape.S +
                                   bary(2) * donut_shape.S;
    aphys::Vecxd interpolate_rotate = bary(0) * twist1_shape.rotate_angle +
                                      bary(1) * twist2_shape.rotate_angle +
                                      bary(2) * donut_shape.rotate_angle;
    aphys::ARAPTargetShape2D::recover_deformation_map(
        interpolated_S, interpolate_rotate, target_F);
    arap_solver.solver_static_shape(v_p, face_indices, fixed_pos, target_F);
  };

  auto linear_interpolate = [&](aphys::Vec3d bary) {
    v_p = bary(0) * v_twist1 + bary(1) * v_twist2 + bary(2) * v_donut;
  };

  // ===================== prepare render data =====================
  aphys::Points points = aphys::create_points();
  points.point_size = 3.0f;

  aphys::Edges edges = aphys::create_edges();
  edges.color = aphys::RGB(0, 0, 0);
  edges.width = 1.0f;

  aphys::Edges weight_triangle_edges = aphys::create_edges();
  weight_triangle_edges.color = aphys::RGB(0, 0, 0);
  weight_triangle_edges.width = 2.0f;

  aphys::MatxXd shape0 = v_twist1 * 0.4;
  shape0.rowwise() += interpolate_triangle.row(0);
  aphys::MatxXd shape1 = v_twist2 * 0.4;
  shape1.rowwise() += interpolate_triangle.row(1);
  aphys::MatxXd shape2 = v_donut * 0.4;
  shape2.rowwise() += interpolate_triangle.row(2);

  aphys::Edges shape0_edges = aphys::create_edges();
  shape0_edges.color = aphys::RGB(0, 0, 255);
  shape0_edges.alpha = 0.5f;
  aphys::Edges shape1_edges = aphys::create_edges();
  shape1_edges.color = aphys::RGB(0, 0, 255);
  shape1_edges.alpha = 0.5f;
  aphys::Edges shape2_edges = aphys::create_edges();
  shape2_edges.color = aphys::RGB(0, 0, 255);
  shape2_edges.alpha = 0.5f;

  aphys::Points weight_point = aphys::create_points();
  weight_point.color = aphys::RGB(212, 0, 34);
  weight_point.point_size = 5.0f;

  aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
  aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());

  aphys::set_edges_data(shape0_edges, shape0.cast<float>(), edge_indices,
                        aphys::MatxXf());
  aphys::set_edges_data(shape1_edges, shape1.cast<float>(), edge_indices,
                        aphys::MatxXf());
  aphys::set_edges_data(shape2_edges, shape2.cast<float>(), edge_indices,
                        aphys::MatxXf());
  aphys::set_edges_data(weight_triangle_edges,
                        interpolate_triangle.cast<float>(), interpolate_edges,
                        aphys::MatxXf());

  aphys::add_render_func(scene, aphys::get_render_func(weight_point));
  // aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));
  aphys::add_render_func(scene, aphys::get_render_func(weight_triangle_edges));
  aphys::add_render_func(scene, aphys::get_render_func(shape0_edges));
  aphys::add_render_func(scene, aphys::get_render_func(shape1_edges));
  aphys::add_render_func(scene, aphys::get_render_func(shape2_edges));

  bool checkboxes[3] = {true, false, false};
  int interpolation_mode = 0;

  aphys::add_gui_func(gui, [&]() {
    if (ImGui::Checkbox("Linear", &checkboxes[0])) {
      interpolation_mode = 0;
      checkboxes[0] = true;
      checkboxes[1] = false;
      checkboxes[2] = false;
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Raw ARAP", &checkboxes[1])) {
      interpolation_mode = 1;
      checkboxes[0] = false;
      checkboxes[1] = true;
      checkboxes[2] = false;
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("ARAP", &checkboxes[2])) {
      interpolation_mode = 2;
      checkboxes[0] = false;
      checkboxes[1] = false;
      checkboxes[2] = true;
    }
  });

  aphys::add_gui_mouse_input_func(gui, [&]() {
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      ImVec2 pos = ImGui::GetMousePos();
      double x = pos.x / SCR_WIDTH;
      x = left + x * (right - left);
      double y = 1.0 - pos.y / SCR_HEIGHT;
      y = bottom + y * (top - bottom);
      aphys::Vec2d target(x, y);
      aphys::Vec3d bary = set_interpolation(target);

      if (interpolation_mode == 0)
        linear_interpolate(bary);
      else if (interpolation_mode == 1)
        raw_arap_interpolate(bary);
      else if (interpolation_mode == 2)
        arap_interpolate(bary);

      aphys::Vecxd mc = aphys::getMassCenter(v_p, vert_mass);
      v_p.rowwise() -= mc.transpose();

      aphys::set_points_data(weight_point, weight_pos.cast<float>(),
                             aphys::MatxXf());
      aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
      aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                            aphys::MatxXf());
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
