#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "igl/readOBJ.h"
#include "igl/biharmonic_coordinates.h"
#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/data/voxel_builder.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;

int main() {
  GLFWwindow *window = aphys::create_window(
      SCR_WIDTH, SCR_HEIGHT, "Example15: Mesh Decimante and Tetrahedralize");

  aphys::DiffuseMaterial material{
      {232, 141, 103},  // diffuse color
      {255, 255, 255},  // specular color
      0.5f              // specular strength
  };
  aphys::DiffuseMaterial tet_material{
      {0, 200, 200},  // diffuse color
      {0, 0, 0},      // specular color
      0.0f            // specular strength
  };
  aphys::Scene scene = aphys::create_scene(
      aphys::Light{
          {242, 242, 242},      // light color
          {100, 100, 100},      // ambient color
          {5.0f, 0.35f, -5.0f}  // light position
      },
      aphys::create_camera(
          {6.0f, 0.0f, -6.0f},                  // camera position
          {0.0f, 0.35f, 0.0f},                  // camera target
          {0.0f, 1.0f, 0.0f},                   // camera up axis
          float(SCR_WIDTH) / float(SCR_HEIGHT)  // camera aspect
          ));

  aphys::Vec3f diffuse_color(0.0f, 211.f / 255.f, 239.f / 255.f);

  aphys::MatxXf V;
  aphys::MatxXi F;
  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);
  int n_vertices = V.rows();
  int n_faces = F.rows();

  // ===================== Voxelize =====================

  double res = 0.2;
  aphys::MatxXd dV = V.cast<double>();

  aphys::VoxelTet voxel_tet(dV, F, res);

  aphys::VisualTetMesh vtm;
  aphys::extract_visual_tets_surfaces(voxel_tet.tets, voxel_tet.verts,
                                      vtm.visual_faces);
  aphys::construct_visual_tets(vtm.visual_verts, voxel_tet.verts,
                               voxel_tet.tets);

  int n_verts = voxel_tet.verts.rows();
  aphys::Vecxd vert_mass;
  aphys::Vecxd tet_mass;
  aphys::compute_tet_mass(voxel_tet.verts, voxel_tet.tets, tet_mass, vert_mass);
  int substep = 2;
  double dt = 1.0f / 60.0f / (double)substep;
  double devia_stiffness = 70.0f;
  double hydro_stiffness = 20.0f;
  double rho = 1000.0f;
  int dim = 3;
  aphys::Vecxd gravity(dim);
  gravity << 0.0f, -4.0f, 0.0f;
  aphys::MatxXd v_vel, v_pred, v_p_ref, v_cache, v_solver;
  v_p_ref = voxel_tet.verts;
  v_vel.resize(n_verts, dim);
  v_vel.setZero();
  v_pred.resize(n_verts, dim);
  v_solver.resize(n_verts, dim);
  aphys::Box3d box(-2.0f, 2.0f, -3.0f, 3.0f, -2.0f, 2.0f);
  aphys::MatxXd external_force;
  aphys::generate_gravity_force(gravity, vert_mass, external_force);
  aphys::ProjectiveDynamicsSolver3D pd_solver(
      voxel_tet.verts, v_p_ref, voxel_tet.tets, tet_mass, vert_mass,
      external_force, dt, hydro_stiffness, devia_stiffness);

  aphys::DiffuseMesh mesh = aphys::create_diffuse_mesh(tet_material);
  mesh.alpha = 0.3;
  aphys::DiffuseMesh fine_mesh = aphys::create_diffuse_mesh(material);

  aphys::Edges box_edges = aphys::create_box_edges();
  box_edges.width = 0.8f;
  box_edges.color = {0, 0, 0};
  aphys::set_box_edges_data(box_edges, box);

  aphys::add_render_func(scene, aphys::get_render_func(mesh), true, true);
  aphys::add_render_func(scene, aphys::get_render_func(fine_mesh));
  aphys::add_render_func(scene, aphys::get_render_func(box_edges));

  aphys::set_wireframe_mode(false);

  while (!glfwWindowShouldClose(window)) {
    aphys::set_background_RGB(aphys::RGB(250, 240, 228));

    v_cache = voxel_tet.verts;
    aphys::ImplicitEuler::predict(v_pred, voxel_tet.verts, v_vel,
                                  external_force, vert_mass, dt);
    for (int i = 0; i < 20; i++) {
      pd_solver.localStep(voxel_tet.verts, voxel_tet.tets);
      pd_solver.globalStep(v_solver, v_pred);
      double error = (voxel_tet.verts - v_solver).norm();
      voxel_tet.verts = v_solver;
      if (error < 1e-4) {
        break;
      }
    }
    aphys::collision3d(box, voxel_tet.verts);
    aphys::ImplicitEuler::updateVelocity(v_vel, voxel_tet.verts, v_cache, dt);

    aphys::interpolate_barycentric(dV, voxel_tet.verts, voxel_tet.tets,
                                   voxel_tet.bind_index,
                                   voxel_tet.bind_weights);

    aphys::construct_visual_tets(vtm.visual_verts, voxel_tet.verts,
                                 voxel_tet.tets, 0.8);
    aphys::set_mesh_data(mesh, vtm.visual_verts.cast<float>(),
                         vtm.visual_faces);
    aphys::set_mesh_data(fine_mesh, dV.cast<float>(), F);

    float curr_time = glfwGetTime();
    aphys::orbit_camera_control(window, scene.camera, 10.0, scene.delta_time);

    aphys::render_scene(scene);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  aphys::delete_mesh(mesh);
  glfwTerminate();

  return 0;
}
