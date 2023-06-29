#ifndef GL_OBJS_H_
#define GL_OBJS_H_

#include <cassert>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "gl_type.h"
#include "gl_buffer.h"
#include "gl_shader.h"
#include "gl_draw.h"
#include "gl_scene.h"

namespace glrender {

struct TexturedMesh {
  VAO vertex_array;
  VBO vertex_buffer;
  VBO uv_buffer;
  EBO index_buffer;
  Texture texture;
  int n_faces;
  int n_vertices;
  Program program;
};

TexturedMesh create_textured_mesh(const Eigen::MatrixXf &vertices,
                                  const Eigen::MatrixXf &texCoords,
                                  const Eigen::MatrixXi &indices,
                                  const ShaderSource &shader,
                                  bool dynamic = false) {
  assert(vertices.IsRowMajor);
  assert(texCoords.IsRowMajor);
  assert(indices.IsRowMajor);

  TexturedMesh mesh{
      create_vao(),
      create_vbo(),
      create_vbo(),
      create_ebo(),
      create_texture(),
      int(indices.rows()),
      int(vertices.rows()),
      create_program(create_shader(shader.vertex, GL_VERTEX_SHADER),
                     create_shader(shader.fragment, GL_FRAGMENT_SHADER))};
  bind_vao(mesh.vertex_array);
  bind_ebo(mesh.index_buffer);
  if (dynamic) {
    set_ebo_dynamic_data(indices.data(), indices.size() * sizeof(int));
  } else {
    set_ebo_static_data(indices.data(), indices.size() * sizeof(int));
  }
  unbind_ebo();
  bind_vbo(mesh.vertex_buffer);
  if (dynamic) {
    set_vbo_dynamic_data(vertices.data(), vertices.size() * sizeof(float));
  } else {
    set_vbo_static_data(vertices.data(), vertices.size() * sizeof(float));
  }
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  unbind_vbo();

  bind_vbo(mesh.uv_buffer);
  if (dynamic) {
    set_vbo_dynamic_data(texCoords.data(), texCoords.size() * sizeof(float));
  } else {
    set_vbo_static_data(texCoords.data(), texCoords.size() * sizeof(float));
  }
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  unbind_vbo();

  unbind_vao();

  return mesh;
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

struct Points {
  std::vector<Vec3f> points;
  std::vector<Vec3f> per_point_color;
  RGB color;
  float point_size;
};

RenderFunc get_render_func(Points &points) {
  RenderFunc render_func = [&](Scene scene) {
    Mat4f projection = scene.camera.projection;
    for (int i = 0; i < points.points.size(); i++) {
      Vec4f pos = projection * Vec4f(points.points[i](0), points.points[i](1),
                                     points.points[i](2), 1.0f);
      pos = pos / pos(3);
      if (points.per_point_color.size() == points.points.size()) {
        draw_point(Vec3f(pos(0), pos(1), pos(2)), points.per_point_color[i],
                   points.point_size);
      } else
        draw_point(Vec3f(pos(0), pos(1), pos(2)), points.color,
                   points.point_size);
    }
  };
  return render_func;
}

struct ContinuousLines {
  std::vector<Vec3f> points;
  std::vector<Vec3f> per_line_color;
  RGB color;
  float line_width;
  bool loop;
};

RenderFunc get_render_func(ContinuousLines &lines) {
  RenderFunc render_func = [&](Scene scene) {
    int diff = 1;
    if (lines.loop) diff = 0;
    for (int i = 0; i < lines.points.size() - diff; i++) {
      int idx1 = i;
      int idx2 = (i + 1) % lines.points.size();
      Vec4f pos_start = scene.camera.projection *
                        Vec4f(lines.points[idx1](0), lines.points[idx1](1),
                              lines.points[idx1](2), 1.0f);
      Vec4f pos_end = scene.camera.projection *
                      Vec4f(lines.points[idx2](0), lines.points[idx2](1),
                            lines.points[idx2](2), 1.0f);
      pos_start = pos_start / pos_start(3);
      pos_end = pos_end / pos_end(3);
      if (lines.per_line_color.size() != 0)
        draw_line(Vec3f(pos_start(0), pos_start(1), pos_start(2)),
                  Vec3f(pos_end(0), pos_end(1), pos_end(2)),
                  lines.per_line_color[i], lines.line_width);
      else
        draw_line(Vec3f(pos_start(0), pos_start(1), pos_start(2)),
                  Vec3f(pos_end(0), pos_end(1), pos_end(2)), lines.color,
                  lines.line_width);
    }
  };
  return render_func;
}

}  // namespace glrender

#endif  // GL_OBJS_H_