#ifndef GL_OBJS_H_
#define GL_OBJS_H_

#include <cassert>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gl_buffer.h"
#include "gl_shader.h"
#include "gl_type.h"

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

}  // namespace glrender

#endif  // GL_OBJS_H_