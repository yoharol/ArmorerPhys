// Use sum-of-squares optimization to find closest point on cubic bezier curve

#include <Eigen/Core>
#include <iostream>
#include <random>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/fem.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

void ClosestPointOnBezier(const aphys::Vecxd& refp, const aphys::MatxXd& verts,
                          const aphys::Matx4i& edges, int& cidx, double& ct) {
  double minDis = 1e10;
  cidx = -1;
  for (int i = 0; i < edges.rows(); i++) {
    aphys::Vecxd p0 = verts.row(edges(i, 0));
    aphys::Vecxd p1 = verts.row(edges(i, 1));
    aphys::Vecxd p2 = verts.row(edges(i, 2));
    aphys::Vecxd p3 = verts.row(edges(i, 3));

    int N = 20;
    for (int j = 0; j <= N; j++) {
      float t = (float)j / N;
      aphys::Vecxd p = aphys::bezier_interpolate(p0, p1, p2, p3, t);
      double dis = (p - refp).norm();
      if (dis < minDis) {
        minDis = dis;
        cidx = i;
        ct = t;
      }
    }
  }
  aphys::Vecxd p0 = verts.row(edges(cidx, 0));
  aphys::Vecxd p1 = verts.row(edges(cidx, 1));
  aphys::Vecxd p2 = verts.row(edges(cidx, 2));
  aphys::Vecxd p3 = verts.row(edges(cidx, 3));
  aphys::Vecxd p = aphys::bezier_interpolate(p0, p1, p2, p3, ct);
  aphys::Vecxd deriv = aphys::bezier_interpolate_deriv(p0, p1, p2, p3, ct);
  aphys::Vecxd second_deriv =
      aphys::bezier_interpolate_second_deriv(p0, p1, p2, p3, ct);
  double d1 = deriv.transpose() * (p - refp);
  double d2 = second_deriv.transpose() * (p - refp) + deriv.squaredNorm();
  ct -= d1 / d2;
}

int main() {
  GLFWwindow* window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                           "Example11: 2D Math Plot with Bezier and B-Spline");
  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, 0.0f, 1.0f, 0.0f, 1.0f);
  aphys::Gui gui = aphys::create_gui(window, "gui");
  gui.width = 300;
  gui.height = 50;

  aphys::MatxXd shape_ref(12, 2);
  shape_ref << 0.4, 0.2, 0.45, 0.15, 0.55, 0.15, 0.6, 0.2, 0.35, 0.35, 0.35,
      0.65, 0.4, 0.8, 0.45, 0.85, 0.55, 0.85, 0.6, 0.8, 0.65, 0.35, 0.65, 0.65;
  aphys::Matx4i edges(4, 4);
  edges << 0, 1, 2, 3,  //
      3, 10, 11, 9,     //
      9, 8, 7, 6,       //
      6, 5, 4, 0;
  aphys::MatxXd shape = shape_ref;

  aphys::MatxXd query_point(2, 2);
  query_point.row(0) = shape_ref.colwise().mean();

  aphys::BezierPeices bp(shape_ref, edges);
  aphys::add_render_func(scene, aphys::get_render_func(bp));
  aphys::Points draw_points = aphys::create_points();
  aphys::add_render_func(scene, aphys::get_render_func(draw_points));

  int curr_idx = 0;
  aphys::add_gui_mouse_input_func(gui, [&]() {
    float x = ImGui::GetMousePos().x / SCR_WIDTH;
    float y = 1.0 - ImGui::GetMousePos().y / SCR_HEIGHT;
    aphys::camera2d_screen_to_world(scene.camera, x, y);
    aphys::Vecxd p(2);
    p << x, y;
    if (ImGui::IsMouseClicked(0)) {
      curr_idx = aphys::find_nearest_point(shape, p);
    }
    if (ImGui::IsMouseDragging(0)) {
      shape.row(curr_idx) = p.transpose();
      bp.set_data(shape, edges);
    }
    if (ImGui::IsMouseDragging(1)) {
      query_point.row(0) = p.transpose();
    }
  });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();
    aphys::handle_gui_input(gui);

    int cidx;
    double ct;
    ClosestPointOnBezier(query_point.row(0), shape, edges, cidx, ct);
    aphys::Vecxd p0 = shape.row(edges(cidx, 0));
    aphys::Vecxd p1 = shape.row(edges(cidx, 1));
    aphys::Vecxd p2 = shape.row(edges(cidx, 2));
    aphys::Vecxd p3 = shape.row(edges(cidx, 3));
    aphys::Vecxd p = aphys::bezier_interpolate(p0, p1, p2, p3, ct);
    query_point.row(1) = p.transpose();

    aphys::set_points_data(draw_points, query_point.cast<float>(),
                           aphys::MatxXf());

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);
    aphys::render_gui(gui);
    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
