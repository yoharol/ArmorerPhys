// triangulate a peice-wide polygon and render it

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "igl/triangle/triangulate.h"
#include "igl/writeTGF.h"
#include "ArmorerPhys/RenderCore.h"
#include "igl/writeDMAT.h"

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

  // ================= create the outside edge of a capsule ==================
  aphys::Vecxd lc(2);
  lc << -0.5, 0.0;
  aphys::Vecxd rc(2);
  rc << 0.5, 0.0;
  double radius = 0.15;

  int n_arch_verts = 20;
  int n_line_verts = 30;
  int n_verts = n_arch_verts * 2 + n_line_verts * 2 - 4;

  aphys::MatxXd v_p(n_verts, 2);
  aphys::Matx2i edge_indices(n_verts, 2);

  int idx = 0;
  for (int i = 0; i < n_arch_verts; i++) {
    double theta = i * M_PI / (n_arch_verts - 1);
    v_p.row(idx) << lc(0) - radius * sin(theta), lc(1) + radius * cos(theta);
    edge_indices.row(idx) << idx, idx + 1;
    idx++;
  }
  for (int i = 1; i < n_line_verts - 1; i++) {
    double d = i * (rc(0) - lc(0)) / (n_line_verts - 1);
    v_p.row(idx) << lc(0) + d, lc(1) - radius;
    edge_indices.row(idx) << idx, idx + 1;
    idx++;
  }
  for (int i = 0; i < n_arch_verts; i++) {
    double theta = i * M_PI / (n_arch_verts - 1);
    v_p.row(idx) << rc(0) + radius * sin(theta), rc(1) - radius * cos(theta);
    edge_indices.row(idx) << idx, idx + 1;
    edge_indices.row(idx) << idx, idx + 1;
    idx++;
  }
  for (int i = 1; i < n_line_verts - 1; i++) {
    double d = i * (rc(0) - lc(0)) / (n_line_verts - 1);
    v_p.row(idx) << rc(0) - d, rc(1) + radius;
    edge_indices.row(idx) << idx, idx + 1;
    idx++;
  }
  edge_indices(idx - 1, 1) = 0;

  // ===================== triangularize =====================

  aphys::MatxXd tri_v_p;
  aphys::Matx3i tri_face_indices;
  aphys::Matx2i tri_edge_indices;

  igl::triangle::triangulate(v_p, edge_indices, aphys::MatxXd(), "a0.001q",
                             tri_v_p, tri_face_indices);
  aphys::extract_edge(tri_face_indices, tri_edge_indices);

  aphys::writeOBJ(std::string(ASSETS_PATH) + "/capsule/capsule.obj", tri_v_p,
                  tri_face_indices);

  int lr_idx = aphys::find_nearest_point(tri_v_p, lc);
  int rr_idx = aphys::find_nearest_point(tri_v_p, rc);
  std::cout << lr_idx << " " << rr_idx << std::endl;
  aphys::MatxXd handles(2, 2);
  handles << tri_v_p(lr_idx, 0), tri_v_p(lr_idx, 1), tri_v_p(rr_idx, 0),
      tri_v_p(rr_idx, 1);
  handles.conservativeResize(handles.rows(), 3);
  handles.col(2).setZero();
  igl::writeTGF(std::string(ASSETS_PATH) + "/capsule/capsule.tgf", handles,
                aphys::MatxXi());

  // ===================== prepare render data =====================
  aphys::Points points = aphys::create_points();
  aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
  points.color = aphys::RGB(255, 0, 0);
  points.point_size = 2.0f;
  aphys::Edges edges = aphys::create_edges();
  aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());
  edges.color = aphys::RGB(0, 0, 0);
  edges.width = 1.0f;

  aphys::Points tri_points = aphys::create_points();
  aphys::set_points_data(tri_points, tri_v_p.cast<float>(), aphys::MatxXf());
  tri_points.color = aphys::RGB(0, 255, 0);
  tri_points.point_size = 2.0f;
  aphys::Edges tri_edges = aphys::create_edges();
  aphys::set_edges_data(tri_edges, tri_v_p.cast<float>(), tri_edge_indices,
                        aphys::MatxXf());
  tri_edges.color = aphys::RGB(0, 0, 0);
  tri_edges.width = 1.0f;

  aphys::set_points_data(points, v_p.cast<float>(), aphys::MatxXf());
  aphys::set_edges_data(edges, v_p.cast<float>(), edge_indices,
                        aphys::MatxXf());
  aphys::set_points_data(tri_points, tri_v_p.cast<float>(), aphys::MatxXf());
  aphys::set_edges_data(tri_edges, tri_v_p.cast<float>(), tri_edge_indices,
                        aphys::MatxXf());

  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));
  aphys::add_render_func(scene, aphys::get_render_func(tri_points));
  aphys::add_render_func(scene, aphys::get_render_func(tri_edges));

  // ===================== main loop =====================
  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    aphys::set_background_RGB({244, 244, 244});

    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
