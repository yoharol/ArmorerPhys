#include <iostream>
#include <Eigen/Core>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/fem.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;

int main() {
  GLFWwindow* window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                           "Example11: 2D Math Plot with Bezier and B-Spline");
  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, -1.0f, 1.0f, -1.0f, 1.0f);

  aphys::MatxXd shape_ref(12, 2);
  shape_ref << 0.4, 0.2, 0.45, 0.15, 0.55, 0.15, 0.6, 0.2, 0.35, 0.35, 0.35,
      0.65, 0.4, 0.8, 0.45, 0.85, 0.55, 0.85, 0.6, 0.8, 0.65, 0.35, 0.65, 0.65;
  aphys::Matx4i edges(4, 4);
  edges << 0, 1, 2, 3, 0, 4, 5, 6, 6, 7, 8, 9, 3, 10, 11, 9;

  aphys::MatxXd shape1 = shape_ref;
  aphys::Mat3d rot = aphys::rotate_around_center({0.5, 0.4}, -M_PI / 3.0);
  for (int i : {5, 6, 7, 8, 9, 11}) {
    aphys::Vec3d homo(shape_ref(i, 0), shape_ref(i, 1), 1.0);
    aphys::Vec3d res = rot * homo;
    shape1.row(i) = res.head(2).transpose();
  }

  aphys::BezierSegment bp1(shape1, edges.row(0));
  aphys::add_render_func(scene, aphys::get_render_func(bp1.spline));
  aphys::BezierSegment bp2(shape1, edges.row(1));
  aphys::add_render_func(scene, aphys::get_render_func(bp2.spline));
  aphys::BezierSegment bp3(shape1, edges.row(2));
  aphys::add_render_func(scene, aphys::get_render_func(bp3.spline));
  aphys::BezierSegment bp4(shape1, edges.row(3));
  aphys::add_render_func(scene, aphys::get_render_func(bp4.spline));

  aphys::MatxXd shape2 = shape_ref;
  aphys::Mat3d rot2 = aphys::rotate_around_center({0.5, 0.4}, M_PI / 3.0);
  for (int i : {5, 6, 7, 8, 9, 11}) {
    aphys::Vec3d homo(shape_ref(i, 0), shape_ref(i, 1), 1.0);
    aphys::Vec3d res = rot2 * homo;
    shape2.row(i) = res.head(2).transpose();
  }
  shape2.rowwise() -= aphys::Vec2d(1.0, 0.0).transpose();

  aphys::BezierSegment bp5(shape2, edges.row(0));
  aphys::add_render_func(scene, aphys::get_render_func(bp5.spline));
  aphys::BezierSegment bp6(shape2, edges.row(1));
  aphys::add_render_func(scene, aphys::get_render_func(bp6.spline));
  aphys::BezierSegment bp7(shape2, edges.row(2));
  aphys::add_render_func(scene, aphys::get_render_func(bp7.spline));
  aphys::BezierSegment bp8(shape2, edges.row(3));
  aphys::add_render_func(scene, aphys::get_render_func(bp8.spline));

  aphys::Matx3i virtual_faces(8, 3);
  for (int i = 0; i < edges.rows(); i++) {
    virtual_faces.row(i * 2) << edges(i, 0), edges(i, 1), edges(i, 2);
    virtual_faces.row(i * 2 + 1) << edges(i, 1), edges(i, 2), edges(i, 3);
  }

  aphys::Matx2i virtual_edges;
  aphys::extract_edge(virtual_faces, virtual_edges);
  aphys::Edges redge = aphys::create_edges();
  aphys::set_edges_data(redge, shape1.cast<float>(), virtual_edges,
                        aphys::MatxXf());
  aphys::add_render_func(scene, aphys::get_render_func(redge));

  aphys::MatxXd source_B;
  aphys::NeoHookeanFEM2D::project_B(shape1, shape_ref, virtual_faces, source_B);
  aphys::ARAPTargetShape2D as1(shape1, virtual_faces, source_B);
  aphys::ARAPTargetShape2D as2(shape2, virtual_faces, source_B);

  aphys::MatxXd target_F = source_B;
  aphys::Vecxi fixed_verts(1);
  fixed_verts << 0;
  aphys::MatxXd fixed_pos(1, 2);
  fixed_pos << 0.0, -0.5;
  aphys::Vecxd face_mass, vert_mass;
  aphys::MatxXd external_force;
  aphys::compute_mesh_mass(shape_ref, virtual_faces, face_mass, vert_mass,
                           1000.0);
  aphys::Vecxd gravity(2);
  gravity << 0.0, 0.0;
  aphys::generate_gravity_force(gravity, vert_mass, external_force);
  aphys::MatxXd v_p = shape_ref;

  aphys::ARAPInterpolate2D arap_solver(v_p, shape_ref, virtual_faces, face_mass,
                                       vert_mass, external_force, 1.0, 1.0,
                                       fixed_verts);
  aphys::MatxXd interpolated_S(2, virtual_faces.rows() * 2);

  aphys::BezierSegment bp9(v_p, edges.row(0));
  aphys::add_render_func(scene, aphys::get_render_func(bp9.spline));
  aphys::BezierSegment bp10(v_p, edges.row(1));
  aphys::add_render_func(scene, aphys::get_render_func(bp10.spline));
  aphys::BezierSegment bp11(v_p, edges.row(2));
  aphys::add_render_func(scene, aphys::get_render_func(bp11.spline));
  aphys::BezierSegment bp12(v_p, edges.row(3));
  aphys::add_render_func(scene, aphys::get_render_func(bp12.spline));

  auto arap_interpolate = [&](double w) {
    interpolated_S = w * as1.S + (1.0 - w) * as2.S;
    aphys::Vecxd interpolate_rotate =
        w * as1.rotate_angle + (1.0 - w) * as2.rotate_angle;
    aphys::ARAPTargetShape2D::recover_deformation_map(
        interpolated_S, interpolate_rotate, target_F);
    arap_solver.solver_static_shape(v_p, virtual_faces, fixed_pos, target_F);
    bp9.set_data(v_p, edges.row(0));
    bp10.set_data(v_p, edges.row(1));
    bp11.set_data(v_p, edges.row(2));
    bp12.set_data(v_p, edges.row(3));
    aphys::set_edges_data(redge, v_p.cast<float>(), virtual_edges,
                          aphys::MatxXf());
  };

  arap_interpolate(0.5);

  // try laplacian smooth on high order mesh

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_DEPTH_TEST);
    glfwPollEvents();

    double time = glfwGetTime();
    double w = 0.5 + 0.5 * sin(time);
    arap_interpolate(w);

    aphys::set_background_RGB({244, 244, 244});
    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}
