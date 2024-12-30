

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <igl/readOBJ.h>
#include <iostream>

#include "ArmorerPhys/DataCore.h"
#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"
#include "ArmorerPhys/render/shader.h"
#include "ArmorerPhys/sim/pd.h"

const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 1000;

int main() {
  using namespace aphys;

  GLFWwindow *window = create_window(
      SCR_WIDTH, SCR_HEIGHT, "Example24: 3D Projective Dynamics with Shell");
  DiffuseMaterial material{{90, 178, 255}, {202, 244, 255}, 0.5f};
  DiffuseMaterial sphere_mat{{96, 96, 96}, {233, 233, 233}, 0.5f};
  Scene scene = create_scene(
      Light{{242, 242, 242}, {109, 105, 188}, {2.0f, 0.35f, -5.0f}},
      create_camera({-5.0f, 0.5f, -5.0f}, {0.0f, 0.5f, 0.0f},
                    {0.0f, 1.0f, 0.0f}, float(SCR_WIDTH) / float(SCR_HEIGHT)));
  Gui gui = create_gui(window, "gui");
  gui.width = 300;
  gui.height = 100;

  ClothMesh mesh;

  int nx = 100;
  int ny = 100;
  create_rectangle(-1.2, 1.2, nx, -1.2, 1.2, ny, mesh.verts, mesh.faces);
  MatxXd v_p(mesh.verts.rows(), 3);
  v_p.col(0) = mesh.verts.col(0);
  v_p.col(2) = mesh.verts.col(1);

  // add a sinwave to mesh.verts.col(1) based on mesh.verts.col(0)
  for (int i = 0; i < mesh.verts.rows(); i++) {
    v_p(i, 1) = 1.1;
    // v_p(i, 1) = 1.1 + 0.15 * sin(6.0 * v_p(i, 0) * 3.1415926);
  }
  mesh.verts = v_p;

  mesh.Initialize();

  Vecxd gravity(3);
  gravity << 0.0, -0.4, 0.0;
  MatxXd external_force;
  generate_gravity_force(gravity, mesh.verts_mass, external_force);

  MatxXd v_pred = v_p;
  MatxXd v_cache = v_p;
  MatxXd v_vel = v_p;
  v_vel.setZero();

  double dt = 1.0 / 60.0;
  ProjectiveCloth pd_cloth(mesh, dt, 1e-4, 100.0);

  DiffuseMesh render_mesh = create_diffuse_mesh(material);
  set_mesh_data(render_mesh, v_p.cast<float>(), mesh.faces);
  add_render_func(scene, get_render_func(render_mesh));

  Box3d box(-2.0, 2.0, 0.0, 2.0, -2.0, 2.0);
  Edges render_edges = create_edges();
  set_box_edges_data(render_edges, box);
  add_render_func(scene, get_render_func(render_edges));
  Sphere3d sphere(Vec3d(0.0, 0.0, 0.0), 0.7);
  DiffuseMesh render_sphere = create_diffuse_mesh(sphere_mat);
  set_mesh_data(render_sphere, sphere.V.cast<float>(), sphere.F);
  add_render_func(scene, get_render_func(render_sphere));

  while (!glfwWindowShouldClose(window)) {
    set_background_RGB(RGB(250, 240, 228));

    v_cache = v_p;
    ImplicitEuler::predict(v_pred, v_p, v_vel, external_force, mesh.verts_mass,
                           dt);
    for (int i = 0; i < 15; i++) {
      pd_cloth.localStep(v_p, mesh);
      pd_cloth.globalStep(v_p, v_pred);
    }
    collision3d(sphere, v_p);
    collision3d(box, v_p);
    ImplicitEuler::updateVelocity(v_vel, v_p, v_cache, dt);

    set_mesh_data(render_mesh, v_p.cast<float>(), mesh.faces);

    orbit_camera_control(window, scene.camera, 10.0, scene.delta_time);
    render_scene(scene);
    render_gui(gui);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwTerminate();
}
