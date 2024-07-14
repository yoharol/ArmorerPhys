// bounded biharmonic weights

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "igl/readTGF.h"
#include "igl/readDMAT.h"
#include "igl/boundary_conditions.h"
#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

double cubicSplineKernel(double d, double h) {
  double q = d / h;
  double W = 0.0;

  if (q >= 0 && q <= 1) {
    W = 1 - 3 * std::pow(q, 2) + 2 * std::pow(q, 3);
  }

  return W;
}

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
  // convert v_p tp 2d
  v_p = v_p.block(0, 0, v_p.rows(), 2);
  aphys::extract_edge(face_indices, edge_indices);
  Eigen::MatrixXd read_handles;
  Eigen::MatrixXi read_handle_edges;
  igl::readTGF(std::string(ASSETS_PATH) + "/capsule/capsule.tgf", read_handles,
               read_handle_edges);
  handles = read_handles;
  aphys::MatxXd tmp = handles.block(0, 0, handles.rows(), 2);
  handles = tmp;
  handle_edges = read_handle_edges;

  aphys::MatxXd weights;
  igl::readDMAT(std::string(ASSETS_PATH) + "/capsule/capsule.dmat", weights);

  // ===================== prepare sim data =====================

  // ===================== prepare simulation data =====================
  double dt = 1.0 / 60.0;
  double devia_stiffness = 10.0f;
  double hydro_stiffness = 10.0f;
  double rho = 1000.0f;
  int dim = 2;
  aphys::Vecxd gravity(dim);
  gravity << 0.0f, 0.0f;
  aphys::MatxXd v_p_ref;
  aphys::MatxXd v_rig;
  v_p_ref = v_p;
  aphys::Vecxd vert_mass, face_mass;
  aphys::compute_mesh_mass(v_p_ref, face_indices, face_mass, vert_mass, rho);
  aphys::MatxXd external_force;
  aphys::generate_gravity_force(gravity, vert_mass, external_force);

  aphys::ProjectiveDynamicsSolver2D pd_solver(
      v_p, v_p_ref, face_indices, face_mass, vert_mass, external_force, dt,
      hydro_stiffness, devia_stiffness);
  int n_fix = 2;
  std::vector<int> fixed_indices(2);
  fixed_indices[0] = 253;
  fixed_indices[1] = 277;
  aphys::SparseMatd sp = pd_solver.L;
  sp.conservativeResize(sp.rows() + n_fix, sp.cols() + n_fix);
  for (int i = 0; i < n_fix; i++) {
    sp.insert(pd_solver.L.rows() + i, fixed_indices[i]) = 1.0;
    sp.insert(fixed_indices[i], pd_solver.L.cols() + i) = 1.0;
  }
  Eigen::SparseLU<aphys::SparseMatd, Eigen::COLAMDOrdering<int>> sparse_solver;
  sparse_solver.analyzePattern(sp);
  sparse_solver.factorize(sp);
  aphys::MatxXd dx = v_p;
  aphys::MatxXd residual_force = v_p;

  // ===================== lbs =====================
  std::vector<aphys::MatxXd> T;
  for (int i = 0; i < handles.rows(); i++) {
    aphys::MatxXd t(2, 3);
    t << 1, 0, 0, 0, 1, 0;
    T.push_back(t);
  }
  aphys::MatxXd v_p_rig = v_p;
  aphys::MatxXd handles_rig = handles;
  aphys::MatxXd A(v_p.rows() * 2, 3);

  auto lbs = [&]() {
    for (int i = 0; i < v_p.rows(); i++) {
      aphys::MatxXd t(2, 3);
      for (int j = 0; j < T.size(); j++) {
        t += weights(i, j) * T[j];
      }
      A.block(i * 2, 0, 2, 3) = t;
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

  aphys::SparseMatd lbs_W(v_p.rows() * 2, handles.rows() * 6);
  auto construct_W = [&]() {
    lbs_W.resize(0, 0);
    lbs_W.resize(v_p.rows() * 2, handles.rows() * 6);
    for (int i = 0; i < v_p.rows(); i++) {
      for (int j = 0; j < handles.rows(); j++) {
        double w = weights(i, j);
        lbs_W.insert(2 * i, j * 6 + 0) = w * v_p_ref(i, 0);
        lbs_W.insert(2 * i, j * 6 + 1) = w * v_p_ref(i, 1);
        lbs_W.insert(2 * i, j * 6 + 2) = w;
        lbs_W.insert(2 * i + 1, j * 6 + 3) = w * v_p_ref(i, 0);
        lbs_W.insert(2 * i + 1, j * 6 + 4) = w * v_p_ref(i, 1);
        lbs_W.insert(2 * i + 1, j * 6 + 5) = w;
      }
    }
    lbs_W.makeCompressed();
  };
  construct_W();

  // ===================== optimize =====================
  int hp_idx;
  aphys::MatxXd hp_pos(1, 2);
  aphys::Matx3f hp_weight_color(v_p.rows(), 3);
  aphys::MatxXd optimized_rig = v_p_rig;
  auto optimize_one_step = [&]() {
    aphys::MatxXd delta_x = dx - v_p_rig;
    double max_dis = -1.0;
    for (int i = 0; i < delta_x.rows(); i++) {
      double dis = (dx.row(i) - v_p_rig.row(i)).squaredNorm();
      if (dis > max_dis) {
        max_dis = dis;
        hp_idx = i;
      }
    }
    hp_pos = v_p_rig.row(hp_idx);

    aphys::Vecxd translate = dx.row(hp_idx) - v_p_ref.row(hp_idx);
    aphys::MatxXd P(2, 2), Q(2, 2);
    P.setZero();
    Q.setZero();

    aphys::Vecxd hp_weights(delta_x.rows());

    for (int i = 0; i < delta_x.rows(); i++) {
      hp_weights(i) = cubicSplineKernel(
          (v_p_ref.row(i) - v_p_ref.row(hp_idx)).norm(), 0.30);
      aphys::Vecxd color = aphys::heat_rgb(hp_weights(i), 0.0, 1.0);
      hp_weight_color.row(i) = color.cast<float>();
    }

    for (int i = 0; i < delta_x.rows(); i++) {
      aphys::Vecxd pi = dx.row(i) - dx.row(hp_idx);
      aphys::Vecxd qi = v_p_ref.row(i) - v_p_ref.row(hp_idx);
      double w = hp_weights(i);
      P += w * pi * qi.transpose();
      Q += w * qi * qi.transpose();
    }

    aphys::MatxXd hp_A(2, 2);
    hp_A = P * Q.inverse();
    hp_A.conservativeResize(2, 3);
    hp_A.block(0, 2, 2, 1) = translate;

    std::cout << hp_A << std::endl;

    set_handle_world_affine(hp_A, v_p_ref.row(hp_idx));

    hp_A = 2.0 * hp_A - A.block(hp_idx * 2, 0, 2, 3);

    for (int i = 0; i < delta_x.rows(); i++) {
      double weight_sum = 1.0 + hp_weights(i);
      aphys::MatxXd t(2, 3);
      t = (A.block(i * 2, 0, 2, 3) + hp_weights[i] * hp_A) / weight_sum;
      aphys::Vecxd x_tilde(3);
      x_tilde << v_p_ref(i, 0), v_p_ref(i, 1), 1;
      optimized_rig.row(i) = (t * x_tilde).transpose();
    }
  };

  // ===================== prepare render data =====================
  aphys::Points points = aphys::create_points();
  points.point_size = 3.0f;

  aphys::Points handle_points = aphys::create_points();
  handle_points.color = aphys::RGB(0, 0, 0);
  handle_points.point_size = 5.0f;

  aphys::Edges edges = aphys::create_edges();
  edges.color = aphys::RGB(255, 0, 0);
  edges.width = 0.5f;

  aphys::Lines force_lines = aphys::create_deriv_lines();
  force_lines.color = aphys::RGB(220, 0, 131);
  force_lines.width = 1.0f;

  aphys::Points hp_points = aphys::create_points();
  hp_points.color = aphys::RGB(0, 255, 0);
  hp_points.point_size = 5.0f;

  aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
  aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());
  aphys::set_points_data(handle_points, handles.cast<float>(), aphys::MatxXf());

  aphys::add_render_func(scene, aphys::get_render_func(handle_points));
  aphys::add_render_func(scene, aphys::get_render_func(hp_points));
  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(force_lines));
  aphys::add_render_func(scene, aphys::get_render_func(edges));

  // ===================== handle input =====================
  int curr_mode = 0;
  aphys::add_gui_key_input_func(gui,
                                [&curr_mode, &handles, &optimize_one_step]() {
                                  if (ImGui::IsKeyPressed(ImGuiKey_H)) {
                                    curr_mode = (curr_mode + 1) % 3;
                                  }
                                  if (ImGui::IsKeyPressed(ImGuiKey_R)) {
                                    optimize_one_step();
                                  }
                                });

  // ===================== main loop =====================
  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    aphys::handle_gui_input(gui);
    aphys::set_background_RGB({244, 244, 244});

    double t = scene.time * 0.5;
    // double angle_l = std::pow(std::sin(t * M_PI), 2.0) * M_PI / 3.0;
    double angle_l = M_PI / 2.5;
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
    /*aphys::Vecxd v_vec = aphys::Vecxd::Map(v_p_ref.data(), v_p_ref.size());
    aphys::Vecxd T_vec(handles.rows() * 6);
    for (int j = 0; j < handles.rows(); j++) {
      aphys::Vecxd t_vec = aphys::Vecxd::Map(T[j].data(), T[j].size());
      T_vec.segment(j * 6, 6) = t_vec;
    }
    v_vec = lbs_W * T_vec;
    v_p_rig = aphys::MatxXd::Map(v_vec.data(), v_p.rows(), 2);*/

    pd_solver.localStep(v_p_rig, face_indices);
    aphys::MatxXd tmp = pd_solver.J * pd_solver.P;
    tmp.conservativeResize(tmp.rows() + n_fix, tmp.cols());
    for (int i = 0; i < n_fix; i++) {
      tmp.row(tmp.rows() - n_fix + i) = v_p_rig.row(fixed_indices[i]);
    }
    Eigen::MatrixXd result = sparse_solver.solve(tmp);
    dx = result.topRows(v_p.rows());

    residual_force = pd_solver.L * v_p_rig - tmp;

    // aphys::set_deriv_lines_data(force_lines, v_p_rig.cast<float>(),
    //                             dx.cast<float>(), 1.0f, aphys::MatxXf());
    if (curr_mode == 0) {
      aphys::set_points_data(points, v_p_rig.cast<float>(), aphys::MatxXf());
      aphys::set_edges_data(edges, v_p_rig.cast<float>(), edge_indices,
                            aphys::MatxXf());
    } else if (curr_mode == 1) {
      aphys::set_points_data(points, dx.cast<float>(), aphys::MatxXf());
      aphys::set_edges_data(edges, dx.cast<float>(), edge_indices,
                            aphys::MatxXf());
    } else if (curr_mode == 2) {
      aphys::set_edges_data(edges, optimized_rig.cast<float>(), edge_indices,
                            aphys::MatxXf());
      aphys::set_points_data(points, optimized_rig.cast<float>(),
                             hp_weight_color);
    }
    aphys::set_points_data(handle_points, handles_rig.cast<float>(),
                           aphys::MatxXf());
    aphys::set_points_data(hp_points, hp_pos.cast<float>(), aphys::MatxXf());

    aphys::render_scene(scene);
    aphys::render_gui(gui);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
