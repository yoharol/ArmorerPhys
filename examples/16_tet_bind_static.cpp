#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "igl/readOBJ.h"
#include "igl/readMESH.h"
#include "igl/writeDMAT.h"
#include "ArmorerPhys/data/tetrahetralize.h"

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/data/voxel_builder.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;

int main() {
  GLFWwindow *window = aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example16: Adjust tet to mesh");

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

  aphys::MatxXd V0;
  aphys::MatxXi F0;
  igl::readOBJ(std::string(ASSETS_PATH) + "/dino/dino.obj", V0, F0);
  V0 = V0 * 0.3;

  aphys::TetMesh tm;
  aphys::VisualTetMesh vtm;

  igl::readMESH(std::string(ASSETS_PATH) + "/dino/dino.mesh", tm.verts, tm.tets,
                tm.faces);

  aphys::extract_visual_tets_surfaces(tm.tets, tm.verts, vtm.visual_faces);

  aphys::MatxXd bc_weights;
  aphys::Vecxi bc_index;
  aphys::bind_to_tet(V0, tm, bc_index, bc_weights);
  aphys::SparseMatd bind_mat;
  aphys::generate_bind_mat(tm.verts.rows(), tm.tets, bc_index, bc_weights,
                           bind_mat);
  igl::writeDMAT(std::string(ASSETS_PATH) + "/dino/dino_weight_idx.dmat",
                 bc_index);
  igl::writeDMAT(std::string(ASSETS_PATH) + "/dino/dino_weight_weight.dmat",
                 bc_weights);

  // ===================== Simulation =====================

  int n_verts = tm.verts.rows();
  aphys::Vecxd vert_mass;
  aphys::Vecxd tet_mass;
  aphys::compute_tet_mass(tm.verts, tm.tets, tet_mass, vert_mass);
  double devia_stiffness = 70.0f;
  double hydro_stiffness = 70.0f;
  double rho = 1000.0f;
  int dim = 3;
  aphys::Vecxd gravity(dim);
  gravity << 0.0f, 0.0f, 0.0f;
  aphys::MatxXd v_vel, v_pred, v_p_ref, v_cache, v_solver;
  v_p_ref = tm.verts;
  v_vel.resize(n_verts, dim);
  v_vel.setZero();
  v_pred.resize(n_verts, dim);
  v_solver.resize(n_verts, dim);
  aphys::Box3d box(-2.0f, 2.0f, -3.0f, 3.0f, -2.0f, 2.0f);
  aphys::MatxXd external_force;
  aphys::generate_gravity_force(gravity, vert_mass, external_force);
  aphys::StaticTetBindPDSolver static_pd(tm, v_p_ref, vert_mass, tet_mass,
                                         external_force, hydro_stiffness,
                                         devia_stiffness, bind_mat);

  aphys::DiffuseMesh mesh = aphys::create_diffuse_mesh(tet_material);
  aphys::DiffuseMesh fine_mesh = aphys::create_diffuse_mesh(material);

  aphys::add_render_func(scene, aphys::get_render_func(mesh), true, false);
  aphys::add_render_func(scene, aphys::get_render_func(fine_mesh), true);

  aphys::MatxXd target_verts;
  int idx = 0;
  auto read_target = [&target_verts](int idx) {
    std::string target_path = std::string(ASSETS_PATH) +
                              "/dino/anim/dino_keyframe_" +
                              std::to_string(idx) + ".obj";
    aphys::MatxXi tmp_F;
    igl::readOBJ(target_path, target_verts, tmp_F);
    target_verts = target_verts * 0.3;
  };
  read_target(idx);

  /*for (int i = 0; i < 11; i++) {
    read_target(i);
    v_cache = tm.verts;
    v_cache.setZero();
    while ((tm.verts - v_cache).squaredNorm() > 1e-6) {
      v_cache = tm.verts;
      static_pd.local_step(tm.verts, tm.tets);
      static_pd.global_step(tm.verts, target_verts, bind_mat);
    }
    std::string export_path = std::string(ASSETS_PATH) +
                              "/dino/anim/dino_tet_" + std::to_string(i) +
                              ".mesh";

    igl::writeMESH(export_path, tm.verts, tm.tets, tm.faces);
  }*/

  while (!glfwWindowShouldClose(window)) {
    aphys::set_background_RGB(aphys::RGB(250, 240, 228));

    v_cache = tm.verts;
    static_pd.local_step(tm.verts, tm.tets);
    static_pd.global_step(tm.verts, target_verts, bind_mat);
    if ((tm.verts - v_cache).squaredNorm() < 1e-6) {
      idx = (idx + 1) % 10;
      read_target(idx);
    }

    V0 = bind_mat * tm.verts;
    V0.col(2) = V0.col(2).array() - 1.0;

    aphys::construct_visual_tets(vtm.visual_verts, tm.verts, tm.tets);
    aphys::set_mesh_data(mesh, vtm.visual_verts.cast<float>(),
                         vtm.visual_faces);
    aphys::set_mesh_data(fine_mesh, V0.cast<float>(), F0);

    aphys::orbit_camera_control(window, scene.camera, 10.0, scene.delta_time);

    aphys::render_scene(scene);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  aphys::delete_mesh(mesh);
  aphys::delete_mesh(fine_mesh);
  glfwTerminate();

  return 0;
}
