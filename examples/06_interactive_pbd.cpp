// interactive position based dynamics with glfw callback input functions

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <memory>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/pbd.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window = aphys::create_window(
      SCR_WIDTH, SCR_HEIGHT, "Example6: interactive pbd simulator");
  double bottom = 0.0f;
  double top = 1.0f;
  double left = 0.0f;
  double right = 1.0f;

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, left, right, bottom, top);

  // ===================== create a rectangle =====================
  aphys::MatxXd v_p;
  aphys::Matx3i face_indices;
  aphys::create_rectangle(0.4, 0.6, 6, 0.2, 0.8, 10, v_p, face_indices);
  int n_verts = v_p.rows();
  aphys::Matx2i edge_indices;
  aphys::extract_edge(face_indices, edge_indices);

  // ===================== prepare simulation data =====================
  aphys::MatxXd v_vel, v_cache;
  aphys::MatxXd v_p_ref = v_p;
  v_vel.resize(v_p.rows(), v_p.cols());
  v_cache.resize(v_p.rows(), v_p.cols());
  aphys::Vecxd rest_length, vert_mass, face_mass, vert_invm;
  aphys::compute_edge_length(v_p, edge_indices, rest_length);
  aphys::compute_mesh_mass(v_p, face_indices, face_mass, vert_mass);
  vert_invm = vert_mass.cwiseInverse();

  // ===================== prepare control point =====================
  aphys::MatxXd v_p_rig = v_p;
  aphys::MatxXd v_control(1, 2);
  double angle = 0.0;
  aphys::MatxXd v_control_ref;
  v_control << 0.5, 0.5;
  v_control_ref = v_control;
  double angle_ref = 0.0;
  aphys::Vecxd Tvec(v_control.rows() * 6);

  auto update_control = [&]() {
    aphys::MatxXd R(2, 2);
    double dangle = angle - angle_ref;
    R << cos(dangle), -sin(dangle), sin(dangle), cos(dangle);
    aphys::Vecxd t(2);
    t = v_control.row(0).transpose() - R * v_control_ref.row(0).transpose();
    aphys::MatxXd affine(2, 3);
    affine.block(0, 0, 2, 2) = R;
    affine.block(0, 2, 2, 1) = t;
    Tvec = Eigen::Map<aphys::Vecxd>(affine.data(), affine.size());
  };

  aphys::MatxXd v_control_weights(2, v_p.rows());
  aphys::SparseMatd lbs_W(v_p.rows() * 2, v_control.rows() * 6);

  double mass_sum = vert_mass.sum();
  for (int i = 0; i < v_p.rows(); i++) {
    double w = 1.0;
    lbs_W.insert(2 * i, 0) = w * v_p_ref(i, 0);
    lbs_W.insert(2 * i, 1) = w * v_p_ref(i, 1);
    lbs_W.insert(2 * i, 2) = w;
    lbs_W.insert(2 * i + 1, 3) = w * v_p_ref(i, 0);
    lbs_W.insert(2 * i + 1, 4) = w * v_p_ref(i, 1);
    lbs_W.insert(2 * i + 1, 5) = w;
  }
  lbs_W.makeCompressed();
  auto lbs = [&]() {
    aphys::Vecxd rig_vec = lbs_W * Tvec;
    v_p_rig = aphys::MatxXd::Map(rig_vec.data(), n_verts, 2);
  };

  update_control();
  lbs();
  // ===================== prepare simulation =====================
  double dt = 1.0 / 60.0 / 15.0;
  aphys::Vecxd gravity(2);
  gravity << 0.0f, 0.0f;
  aphys::PbdFramework pbd_framework;
  pbd_framework.addConstraint(std::make_shared<aphys::DeformConstraint<2>>(
      v_p, v_p_ref, face_indices, face_mass, vert_invm, 0.1, 0.1, dt));
  pbd_framework.addConstraint(std::make_shared<aphys::AffineConstraint2D>(
      v_p, v_p_ref, v_p_rig, lbs_W, vert_mass, vert_invm, Tvec, 1e-4, 1e-4,
      dt));
  aphys::Box2d box(0.0f, 1.0f, 0.0f, 1.0f);

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

  aphys::Edges rig_edges = aphys::create_edges();
  aphys::set_edges_data(rig_edges, v_p_rig.cast<float>(), edge_indices,
                        aphys::MatxXf());
  rig_edges.color = aphys::RGB(0, 0, 255);
  rig_edges.alpha = 0.3f;

  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));
  aphys::add_render_func(scene, aphys::get_render_func(rig_edges));

  // ===================== handle input =====================
  aphys::InputHandler& handler = aphys::create_input_handler(window);
  aphys::add_mouse_move_func(handler, [&](aphys::InputHandler& input_handler) {
    if (input_handler.left_pressing) {
      double xpos, ypos;
      xpos = left + (right - left) * handler.xpos;
      ypos = bottom + (top - bottom) * handler.ypos;
      v_control.row(0) = aphys::Vec2d(xpos, ypos);
    }
  });
  aphys::add_key_input_func(
      handler, [&](aphys::InputHandler& input_handler, int key, int action) {
        if (key == GLFW_KEY_Q && action == GLFW_REPEAT) {
          angle -= 2.0 * scene.delta_time;
        }
        if (key == GLFW_KEY_E && action == GLFW_REPEAT) {
          angle += 2.0 * scene.delta_time;
        }
      });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    update_control();
    lbs();
    for (int _ = 0; _ < 15; ++_) {
      pbd_framework.pbdPredict(v_p, v_vel, v_cache, vert_invm, gravity, dt);
      pbd_framework.preProjectConstraints();
      pbd_framework.projectConstraints();
      aphys::collision2d(box, v_p);
      pbd_framework.pbdUpdateVelocity(v_p, v_vel, v_cache, dt, 1.0f);
    }

    glfwPollEvents();
    aphys::set_background_RGB({244, 244, 244});

    aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
    aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                          aphys::MatxXf());
    aphys::set_edges_data(rig_edges, v_p_rig.cast<float>(), edge_indices,
                          aphys::MatxXf());

    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}