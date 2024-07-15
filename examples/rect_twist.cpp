// bounded biharmonic weights

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "igl/writeOBJ.h"
#include "ArmorerPhys/RenderCore.h"

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

  // ================= create the outside edge of a capsule ==================
  aphys::MatxXd v_p, v_p_ref;
  aphys::Matx3i face_indices;
  aphys::Matx2i edge_indices;

  aphys::create_rectangle(-2.0, 2.0, 40, -0.2, 0.2, 4, v_p_ref, face_indices);
  v_p = v_p_ref;

  aphys::extract_edge(face_indices, edge_indices);

  aphys::MatxXd sin_turbulated_v_p1 = v_p_ref;
  for (int i = 0; i < sin_turbulated_v_p1.rows(); i++) {
    sin_turbulated_v_p1(i, 1) +=
        0.3 * cos(1.5 * M_PI * sin_turbulated_v_p1(i, 0));
  }

  aphys::MatxXd rot_mat1(2, 2);
  rot_mat1 << 0.7071067811865476, -0.7071067811865475, 0.7071067811865475,
      0.7071067811865476;
  aphys::MatxXd v_p_rot_1 = sin_turbulated_v_p1 * rot_mat1.transpose();

  aphys::MatxXd sin_turbulated_v_p2 = v_p_ref;
  for (int i = 0; i < sin_turbulated_v_p2.rows(); i++) {
    sin_turbulated_v_p2(i, 1) +=
        0.25 * sin(2.0 * M_PI * sin_turbulated_v_p2(i, 0));
  }

  aphys::MatxXd rot_mat2(2, 2);
  rot_mat2 << 0.7071067811865476, 0.7071067811865475, -0.7071067811865475,
      0.7071067811865476;
  aphys::MatxXd v_p_rot_2 = sin_turbulated_v_p2 * rot_mat2.transpose();

  aphys::MatxXd donut_v_p = v_p_ref;
  for (int i = 0; i < donut_v_p.rows(); i++) {
    double r = 4.0 / (2.0 * M_PI);
    double x = r * cos(2.0 * M_PI * donut_v_p(i, 0) / 4.0);
    double y = r * sin(2.0 * M_PI * donut_v_p(i, 0) / 4.0);
    double local_offset = -donut_v_p(i, 1);
    aphys::Vec2d p(x, y);
    donut_v_p.row(i) = p.normalized() * (p.norm() + local_offset);
  }

  // ===================== write to files =====================
  aphys::writeOBJ(std::string(ASSETS_PATH) + "/rect_twist/ref.obj", v_p_ref,
                  face_indices);
  aphys::writeOBJ(std::string(ASSETS_PATH) + "/rect_twist/twist1.obj",
                  v_p_rot_1, face_indices);
  aphys::writeOBJ(std::string(ASSETS_PATH) + "/rect_twist/twist2.obj",
                  v_p_rot_2, face_indices);
  aphys::writeOBJ(std::string(ASSETS_PATH) + "/rect_twist/donut.obj", donut_v_p,
                  face_indices);

  // ===================== prepare render data =====================
  aphys::Points points = aphys::create_points();
  points.point_size = 3.0f;

  aphys::Edges edges = aphys::create_edges();
  edges.color = aphys::RGB(0, 0, 0);
  edges.width = 1.0f;

  aphys::Edges rot_edges_1 = aphys::create_edges();
  rot_edges_1.color = aphys::RGB(0, 128, 179);
  rot_edges_1.width = 1.0f;

  aphys::Edges rot_edges_2 = aphys::create_edges();
  rot_edges_2.color = aphys::RGB(0, 179, 128);
  rot_edges_2.width = 1.0f;

  aphys::Edges donut_edges = aphys::create_edges();
  donut_edges.color = aphys::RGB(179, 0, 128);
  donut_edges.width = 1.0f;

  aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
  aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());
  aphys::set_edges_data(rot_edges_1, v_p_rot_1.cast<float>(), edge_indices,
                        aphys::MatxXf());
  aphys::set_edges_data(rot_edges_2, v_p_rot_2.cast<float>(), edge_indices,
                        aphys::MatxXf());
  aphys::set_edges_data(donut_edges, donut_v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());

  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));
  aphys::add_render_func(scene, aphys::get_render_func(rot_edges_1));
  aphys::add_render_func(scene, aphys::get_render_func(rot_edges_2));
  aphys::add_render_func(scene, aphys::get_render_func(donut_edges));

  // ===================== main loop =====================
  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    aphys::set_background_RGB({244, 244, 244});

    aphys::set_points_data(points, v_p_ref.cast<float>(), aphys::MatxXf());
    aphys::set_edges_data(edges, v_p_ref.cast<float>(), edge_indices,
                          aphys::MatxXf());

    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
