#ifndef GL_WARP_H_
#define GL_WARP_H_

#include <functional>

#include "gl_type.h"
#include "gl_buffer.h"
#include "gl_shader.h"
#include "gl_camera.h"
#include "gl_geom.h"

namespace glrender {

struct Scene;

typedef std::function<void(Scene)> RenderFunc;

struct Scene {
  Light light;
  Camera camera;
  std::vector<RenderFunc> render_funcs;
};

Scene create_scene(Light light, Camera camera) {
  Scene scene;
  scene.light = light;
  scene.camera = camera;
  scene.render_funcs = std::vector<RenderFunc>();
  return scene;
}

struct DiffuseMesh {
  VAO vertex_array;
  VBO vertex_buffer;
  VBO normal_buffer;
  EBO index_buffer;
  Program program;
  DiffuseMaterial material;
  int n_vertices;
  int n_faces;
};

DiffuseMesh create_diffuse_mesh(DiffuseMaterial &material) {
  DiffuseMesh mesh;
  mesh.vertex_array = create_vao();
  mesh.vertex_buffer = create_vbo();
  mesh.normal_buffer = create_vbo();
  mesh.index_buffer = create_ebo();
  mesh.program = create_program(
      create_shader(source::basic_diffuse_shader.vertex, GL_VERTEX_SHADER),
      create_shader(source::basic_diffuse_shader.fragment, GL_FRAGMENT_SHADER));
  mesh.material = material;

  return mesh;
}

void set_mesh_data(DiffuseMesh &mesh, MatXf &V, MatXi &F) {
  mesh.n_vertices = V.rows();
  mesh.n_faces = F.rows();

  use_program(mesh.program);

  bind_vao(mesh.vertex_array);

  bind_ebo(mesh.index_buffer);
  set_ebo_static_data(F.data(), F.size() * sizeof(int));
  unbind_ebo();

  bind_vbo(mesh.vertex_buffer);
  set_vbo_dynamic_data(V.data(), V.size() * sizeof(float));
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  unbind_vbo();

  bind_vbo(mesh.normal_buffer);
  MatXf normals = glrender::get_normals(V, F);
  set_vbo_dynamic_data(normals.data(), normals.size() * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  unbind_vbo();

  set_uniform_RGB(mesh.program, "diffuseColor", mesh.material.diffuse_color);
  set_uniform_RGB(mesh.program, "specularColor", mesh.material.specular_color);
  set_uniform_float(mesh.program, "specularStrength",
                    mesh.material.specular_strength);
  unuse_program();

  unbind_vao();
}

RenderFunc get_render_func(DiffuseMesh &mesh) {
  RenderFunc render_func = [&](Scene scene) {
    use_program(mesh.program);
    set_uniform_mat4(mesh.program, "projection", scene.camera.projection);
    set_uniform_mat4(mesh.program, "view", scene.camera.view);
    set_uniform_RGB(mesh.program, "lightColor", scene.light.light_color);
    set_uniform_RGB(mesh.program, "ambientColor", scene.light.ambient_color);
    set_uniform_float3(mesh.program, "lightPos", scene.light.position);
    set_uniform_float3(mesh.program, "viewPos", scene.camera.position);
    glrender::set_uniform_mat4(mesh.program, "projection",
                               scene.camera.projection);
    glrender::set_uniform_float3(mesh.program, "viewPos",
                                 scene.camera.position);
    glrender::bind_vao(mesh.vertex_array);
    glrender::bind_ebo(mesh.index_buffer);
    glDrawElements(GL_TRIANGLES, mesh.n_faces * 3, GL_UNSIGNED_INT, 0);
    glrender::unbind_vao();
    glrender::unbind_ebo();
    unuse_program();
  };
  return render_func;
}

void delete_mesh(DiffuseMesh &mesh) {
  delete_program(mesh.program);
  delete_vao(mesh.vertex_array);
  delete_vbo(mesh.vertex_buffer);
  delete_vbo(mesh.normal_buffer);
  delete_ebo(mesh.index_buffer);
}

}  // namespace glrender

#endif  // GL_WARP_H_