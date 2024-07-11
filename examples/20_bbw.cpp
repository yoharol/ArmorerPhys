// bounded biharmonic weights

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "igl/readTGF.h"
#include "igl/bbw.h"
#include "igl/writeDMAT.h"
#include "igl/boundary_conditions.h"
#include "ArmorerPhys/RenderCore.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window = aphys::create_window(
      SCR_WIDTH, SCR_HEIGHT, "Example6: interactive pbd simulator");
  double bottom = -1.0f;
  double top = 1.0f;
  double left = -1.0f;
  double right = 1.0f;

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, left, right, bottom, top);

  aphys::Gui gui = aphys::create_gui(window, "gui");
  gui.width = 30;
  gui.height = 10;

  // ================= create the outside edge of a capsule ==================
  aphys::MatxXd handles;
  aphys::MatxXi handle_edges;
  aphys::MatxXd v_p;
  aphys::Matx3i face_indices;
  aphys::Matx2i edge_indices;

  igl::readOBJ(std::string(ASSETS_PATH) + "/capsule/capsule.obj", v_p,
               face_indices);
  aphys::extract_edge(face_indices, edge_indices);
  Eigen::MatrixXd read_handles;
  Eigen::MatrixXi read_handle_edges;
  igl::readTGF(std::string(ASSETS_PATH) + "/capsule/capsule.tgf", read_handles,
               read_handle_edges);
  handles = read_handles;
  handle_edges = read_handle_edges;

  aphys::MatxXd weights;
  igl::BBWData bbw_data;
  bbw_data.active_set_params.max_iter = 8;
  bbw_data.verbosity = 2;

  Eigen::MatrixXd bc;
  Eigen::VectorXi b;
  Eigen::MatrixXd V = v_p;
  Eigen::MatrixXi F = face_indices;
  Eigen::MatrixXd C = handles;
  Eigen::VectorXi P(2);
  P << 0, 1;
  Eigen::MatrixXi BE = handle_edges;
  igl::boundary_conditions(V, F, C, P, BE, Eigen::MatrixXi(), Eigen::MatrixXi(),
                           b, bc);

  if (!igl::bbw(v_p, face_indices, b, bc, bbw_data, weights)) {
    return EXIT_FAILURE;
  }
  for (int i = 0; i < weights.rows(); i++) {
    weights.row(i) /= weights.row(i).sum();
  }

  igl::writeDMAT(std::string(ASSETS_PATH) + "/capsule/capsule.dmat", weights);

  // ===================== prepare render data =====================
  aphys::Points points = aphys::create_points();
  points.point_size = 3.0f;

  aphys::Points handle_points = aphys::create_points();
  handle_points.color = aphys::RGB(0, 0, 0);
  handle_points.point_size = 5.0f;

  aphys::Edges edges = aphys::create_edges();
  edges.color = aphys::RGB(0, 0, 0);
  edges.width = 1.0f;

  aphys::MatxXf weight_color(v_p.rows(), 3);
  auto set_weight_color = [&](int handle_idx) {
    for (int i = 0; i < v_p.rows(); i++) {
      aphys::Vecxf color =
          aphys::heat_rgb(weights(i, handle_idx), 0.0, 1.0).cast<float>();
      weight_color.row(i) = color.transpose();
    }
  };
  set_weight_color(0);

  aphys::set_points_data(points, v_p.cast<float>(), weight_color);
  aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());
  aphys::set_points_data(handle_points, handles.cast<float>(), aphys::MatxXf());

  aphys::add_render_func(scene, aphys::get_render_func(handle_points));
  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));

  // ===================== handle input =====================
  int curr_weight = 0;
  aphys::add_gui_key_input_func(
      gui, [&curr_weight, &handles, &set_weight_color]() {
        if (ImGui::IsKeyPressed(ImGuiKey_H)) {
          curr_weight = (curr_weight + 1) % handles.rows();
          set_weight_color(curr_weight);
        }
      });
  // ===================== lbs =====================
  std::vector<aphys::MatxXd> T;
  for (int i = 0; i < handles.rows(); i++) {
    aphys::MatxXd t(2, 3);
    t << 1, 0, 0, 0, 1, 0;
    T.push_back(t);
  }
  aphys::MatxXd v_p_rig = v_p;
  aphys::MatxXd handles_rig = handles;

  auto lbs = [&]() {
    for (int i = 0; i < v_p.rows(); i++) {
      aphys::MatxXd t(2, 3);
      for (int j = 0; j < T.size(); j++) {
        t += weights(i, j) * T[j];
      }
      aphys::Vecxd x_tilde(3);
      x_tilde << v_p(i, 0), v_p(i, 1), 1;
      v_p_rig.row(i) = (t * x_tilde).transpose();
    }
    for (int j = 0; j < T.size(); j++) {
      aphys::Vecxd x_tilde(3);
      x_tilde << handles(j, 0), handles(j, 1), 1.0;
      handles_rig.row(j) = (T[j] * x_tilde).transpose();
    }
  };

  auto set_handle_world_affine = [&](aphys::MatxXd& local_T,
                                     aphys::Vecxd world_origin_pos) {
    aphys::MatxXd L(3, 3);
    L.setZero();
    L.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    L.block(0, 2, 2, 1) = world_origin_pos;
    L(2, 2) = 1.0;
    aphys::MatxXd C(3, 3);
    C.setZero();
    C.block(0, 0, 2, 3) = local_T;
    C(2, 2) = 1.0;
    aphys::MatxXd R(3, 3);
    R.setZero();
    R.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
    R.block(0, 2, 2, 1) = -world_origin_pos;
    R(2, 2) = 1.0;
    local_T = (L * C * R).block(0, 0, 2, 3);
  };

  // ===================== main loop =====================
  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    aphys::handle_gui_input(gui);
    aphys::set_background_RGB({244, 244, 244});

    double t = scene.time * 0.5;
    double angle_l = std::pow(std::sin(t * M_PI), 2.0) * M_PI / 3.0;
    double angle_r = -angle_l;

    T[0].setZero();
    T[0].block(0, 0, 2, 2) << std::cos(angle_l), -std::sin(angle_l),
        std::sin(angle_l), std::cos(angle_l);
    T[1].setZero();
    T[1].block(0, 0, 2, 2) << std::cos(angle_r), -std::sin(angle_r),
        std::sin(angle_r), std::cos(angle_r);
    set_handle_world_affine(T[0], handles.row(0));
    set_handle_world_affine(T[1], handles.row(1));

    lbs();
    aphys::set_points_data(points, v_p_rig.cast<float>(), weight_color);
    aphys::set_edges_data(edges, v_p_rig.cast<float>(), edge_indices,
                          aphys::MatxXf());
    aphys::set_points_data(handle_points, handles_rig.cast<float>(),
                           aphys::MatxXf());

    aphys::render_scene(scene);
    aphys::render_gui(gui);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
