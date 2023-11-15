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
  float bottom = 0.0f;
  float top = 1.0f;
  float left = 0.0f;
  float right = 1.0f;

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, left, right, bottom, top);

  // ===================== create a rectangle =====================
  aphys::MatxXf v_p;
  aphys::Matx3i face_indices;
  aphys::create_rectangle(0.4, 0.6, 5, 0.2, 0.8, 15, v_p, face_indices);
  aphys::Matx2i edge_indices;
  aphys::extract_edge(face_indices, edge_indices);

  // ===================== prepare simulation data =====================
  aphys::MatxXf v_vel, v_cache;
  aphys::MatxXf v_p_ref = v_p;
  v_vel.resize(v_p.rows(), v_p.cols());
  v_cache.resize(v_p.rows(), v_p.cols());
  aphys::Vecxf rest_length, vert_mass, face_mass, vert_invm;
  aphys::compute_edge_length(v_p, edge_indices, rest_length);
  aphys::compute_mesh_mass(v_p, face_indices, face_mass, vert_mass);
  vert_invm = vert_mass.cwiseInverse();
  vert_invm(0) = 0.0f;

  // ===================== prepare simulation =====================
  float dt = 1.0 / 60.0 / 15.0;
  aphys::Vecxf gravity(2);
  gravity << 0.0f, -0.5f;
  aphys::PbdFramework pbd_framework;
  pbd_framework.addConstraint(std::make_shared<aphys::DeformConstraint<2>>(
      v_p, v_p_ref, face_indices, face_mass, vert_invm, 0.3f, 0.3f, dt));
  aphys::Box2d box(0.0f, 1.0f, 0.0f, 1.0f);

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

  // ===================== handle input =====================
  aphys::InputHandler& handler = aphys::create_input_handler(window);
  aphys::add_mouse_move_func(handler, [&](aphys::InputHandler& input_handler) {
    if (input_handler.left_pressing) {
      float xpos, ypos;
      xpos = left + (right - left) * handler.xpos;
      ypos = bottom + (top - bottom) * handler.ypos;
      v_p.row(0) = aphys::Vec2f(xpos, ypos);
    }
  });
  aphys::add_mouse_input_func(handler, [](aphys::InputHandler& input_handler,
                                          int button, int action) {
    std::cout << "mouse button event: " << button << " " << action << std::endl;
  });
  aphys::add_key_input_func(
      handler, [](aphys::InputHandler& input_handler, int key, int action) {
        std::cout << "key event: " << key << " " << action << std::endl;
      });

  std::cout << "Click and drag mouse to move the fixed point" << std::endl;

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    for (int _ = 0; _ < 15; ++_) {
      pbd_framework.pbdPredict(v_p, v_vel, v_cache, vert_invm, gravity, dt);
      pbd_framework.preProjectConstraints();
      pbd_framework.projectConstraints();
      aphys::collision2d(box, v_p);
      pbd_framework.pbdUpdateVelocity(v_p, v_vel, v_cache, dt, 1.0f);
    }

    glfwPollEvents();
    aphys::set_background_RGB({244, 244, 244});

    aphys::set_points_data(points, v_p, aphys::MatxXf());
    aphys::set_edges_data(edges, v_p, edge_indices, aphys::MatxXf());

    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}