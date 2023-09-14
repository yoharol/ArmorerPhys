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

namespace armgl {

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
  MatXf normals = armgl::get_normals(V, F);
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
    set_uniform_RGB(mesh.program, "lightColor", scene.light.light_color);
    set_uniform_RGB(mesh.program, "ambientColor", scene.light.ambient_color);
    set_uniform_float3(mesh.program, "lightPos", scene.light.position);
    set_uniform_float3(mesh.program, "viewPos", scene.camera.position);
    set_uniform_mat4(mesh.program, "projection", scene.camera.projection);
    set_uniform_float3(mesh.program, "viewPos", scene.camera.position);
    bind_vao(mesh.vertex_array);
    bind_ebo(mesh.index_buffer);
    glDrawElements(GL_TRIANGLES, mesh.n_faces * 3, GL_UNSIGNED_INT, 0);
    unbind_vao();
    unbind_ebo();
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
  int n_points;
  VAO vertex_array;
  VBO vertex_buffer;
  VBO color_buffer;
  Program program;
  RGB color;
  bool uniform_color;
  float point_size;
};

Points create_points() {
  glEnable(GL_PROGRAM_POINT_SIZE);
  return Points{
      0,
      create_vao(),    //
      create_vbo(),    //
      create_vbo(),    //
      create_program(  //
          create_shader(source::point_shader.vertex, GL_VERTEX_SHADER),
          create_shader(source::point_shader.fragment, GL_FRAGMENT_SHADER)),
      RGB(255, 0, 0),  //
      true,            //
      1.0f};
}

void set_points_data(Points &points, const MatXf &points_data,
                     const MatXf &per_point_color) {
  points.n_points = points_data.rows();

  use_program(points.program);

  bind_vao(points.vertex_array);
  bind_vbo(points.vertex_buffer);

  if (points_data.cols() == 2) {
    MatXf points_data_3d(points_data.rows(), 3);
    points_data_3d << points_data, MatXf::Zero(points_data.rows(), 1);
    set_vbo_dynamic_data(points_data_3d.data(),
                         points_data_3d.size() * sizeof(float));
  } else
    set_vbo_dynamic_data(points_data.data(),
                         points_data.size() * sizeof(float));
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  unbind_vbo();

  if (per_point_color.rows() > 0) {
    bind_vbo(points.color_buffer);
    set_vbo_dynamic_data(per_point_color.data(),
                         per_point_color.size() * sizeof(float));
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(1);
    unbind_vbo();
    points.uniform_color = false;
  } else {
    points.uniform_color = true;
  }
  unbind_vao();
  unuse_program();
}

RenderFunc get_render_func(Points &points) {
  RenderFunc render_func = [&](Scene scene) {
    use_program(points.program);
    set_uniform_mat4(points.program, "projection", scene.camera.projection);
    set_uniform_RGB(points.program, "color", points.color);
    set_uniform_float(points.program, "pointSize", points.point_size);
    if (points.uniform_color)
      set_uniform_float(points.program, "choice", 1.0f);
    else
      set_uniform_float(points.program, "choice", 0.0f);
    bind_vao(points.vertex_array);
    glDrawArrays(GL_POINTS, 0, points.n_points);
    unbind_vao();
    unuse_program();
  };
  return render_func;
}

struct Lines {
  int n_points;
  VAO vertex_array;
  VBO vertex_buffer;
  VBO color_buffer;
  Program program;
  RGB color;
  bool uniform_color;
  GLenum mode;
};

Lines create_lines() {
  return Lines{
      0,
      create_vao(),    //
      create_vbo(),    //
      create_vbo(),    //
      create_program(  //
          create_shader(source::line_shader.vertex, GL_VERTEX_SHADER),
          create_shader(source::line_shader.fragment, GL_FRAGMENT_SHADER)),
      RGB(255, 0, 0),  //
      true,
      GL_LINE_STRIP};
}

void set_lines_data(Lines &lines, const MatXf &points_data,
                    const MatXf &per_line_color) {
  lines.n_points = points_data.rows();

  use_program(lines.program);

  bind_vao(lines.vertex_array);
  bind_vbo(lines.vertex_buffer);
  if (points_data.cols() == 2) {
    MatXf points_data_3d(points_data.rows(), 3);
    points_data_3d << points_data, MatXf::Zero(points_data.rows(), 1);
    set_vbo_dynamic_data(points_data_3d.data(),
                         points_data_3d.size() * sizeof(float));
  } else
    set_vbo_dynamic_data(points_data.data(),
                         points_data.size() * sizeof(float));
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  unbind_vbo();

  if (per_line_color.rows() > 0) {
    bind_vbo(lines.color_buffer);
    set_vbo_dynamic_data(per_line_color.data(),
                         per_line_color.size() * sizeof(float));
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(1);
    unbind_vbo();
    lines.uniform_color = false;
  } else {
    lines.uniform_color = true;
  }
  unbind_vao();
  unuse_program();
}

RenderFunc get_render_func(Lines &lines) {
  RenderFunc render_func = [&](Scene scene) {
    use_program(lines.program);
    set_uniform_mat4(lines.program, "projection", scene.camera.projection);
    set_uniform_RGB(lines.program, "color", lines.color);
    if (lines.uniform_color)
      set_uniform_float(lines.program, "choice", 1.0f);
    else
      set_uniform_float(lines.program, "choice", 0.0f);
    bind_vao(lines.vertex_array);
    glDrawArrays(lines.mode, 0, lines.n_points);
    unbind_vao();
    unuse_program();
  };
  return render_func;
}

}  // namespace armgl

#endif  // GL_OBJS_H_