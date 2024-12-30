#include "ArmorerPhys/render/objs.h"

#include <cassert>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/geom.h"
#include "ArmorerPhys/render/buffer.h"
#include "ArmorerPhys/render/shader.h"
#include "ArmorerPhys/render/draw.h"
#include "ArmorerPhys/render/scene.h"
#include "ArmorerPhys/render/window.h"

namespace aphys {

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
  mesh.alpha = 1.0f;

  return mesh;
}

void set_mesh_data(DiffuseMesh &mesh, const MatxXf &V, const MatxXi &F) {
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
  MatxXf normals = aphys::get_normals(V, F);
  set_vbo_dynamic_data(normals.data(), normals.size() * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  unbind_vbo();

  set_uniform_RGB(mesh.program, "diffuseColor", mesh.material.diffuse_color);
  set_uniform_RGB(mesh.program, "specularColor", mesh.material.specular_color);
  set_uniform_float(mesh.program, "specularStrength",
                    mesh.material.specular_strength);
  set_uniform_float(mesh.program, "alpha", mesh.alpha);
  unuse_program();
  unbind_vao();
}

RenderFunc get_render_func(DiffuseMesh &mesh) {
  RenderFunc render_func = [&](Scene &scene) {
    use_program(mesh.program);
    set_uniform_mat4(mesh.program, "projection", scene.camera.projection);
    set_uniform_RGB(mesh.program, "lightColor", scene.light.light_color);
    set_uniform_RGB(mesh.program, "ambientColor", scene.light.ambient_color);
    set_uniform_float3(mesh.program, "lightPos", scene.light.position);
    set_uniform_float3(mesh.program, "viewPos", scene.camera.position);
    set_uniform_mat4(mesh.program, "projection", scene.camera.projection);
    set_uniform_float3(mesh.program, "viewPos", scene.camera.position);
    set_uniform_float(mesh.program, "alpha", mesh.alpha);
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

ColorMesh create_color_mesh(DiffuseMaterial &material) {
  ColorMesh mesh;
  mesh.vertex_array = create_vao();
  mesh.vertex_buffer = create_vbo();
  mesh.normal_buffer = create_vbo();
  mesh.color_buffer = create_vbo();
  mesh.index_buffer = create_ebo();
  mesh.program = create_program(
      create_shader(source::color_diffuse_shader.vertex, GL_VERTEX_SHADER),
      create_shader(source::color_diffuse_shader.fragment, GL_FRAGMENT_SHADER));
  mesh.material = material;
  return mesh;
}

void set_color_mesh_data(ColorMesh &mesh, const MatxXf &V, const MatxXi &F,
                         const MatxXf &C) {
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
  MatxXf normals = aphys::get_normals(V, F);
  set_vbo_dynamic_data(normals.data(), normals.size() * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  unbind_vbo();

  bind_vbo(mesh.color_buffer);
  set_vbo_dynamic_data(C.data(), C.size() * sizeof(float));
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(2);
  unbind_vbo();

  set_uniform_RGB(mesh.program, "specularColor", mesh.material.specular_color);
  set_uniform_float(mesh.program, "specularStrength",
                    mesh.material.specular_strength);
  unuse_program();
  unbind_vao();
}

RenderFunc get_render_func(ColorMesh &mesh) {
  RenderFunc render_func = [&](Scene &scene) {
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

void delete_mesh(ColorMesh &mesh) {
  delete_program(mesh.program);
  delete_vao(mesh.vertex_array);
  delete_vbo(mesh.vertex_buffer);
  delete_vbo(mesh.normal_buffer);
  delete_ebo(mesh.index_buffer);
}

Points create_points() {
  glEnable(GL_PROGRAM_POINT_SIZE);
  return Points{
      0,
      create_vao(),    //
      create_vbo(),    //
      create_vbo(),    //
      create_program(  //
          create_shader(source::point_shader.vertex, GL_VERTEX_SHADER),
          create_shader(source::point_shader.fragment, GL_FRAGMENT_SHADER),
          create_shader(source::point_shader.geometry, GL_GEOMETRY_SHADER)),
      RGB(255, 0, 0),  //
      true,            //
      8.0f};
}

void set_points_data(Points &points, const MatxXf &points_data,
                     const MatxXf &per_point_color) {
  points.n_points = points_data.rows();

  use_program(points.program);

  bind_vao(points.vertex_array);
  bind_vbo(points.vertex_buffer);

  if (points_data.cols() == 2) {
    MatxXf points_data_3d(points_data.rows(), 3);
    points_data_3d << points_data, MatxXf::Zero(points_data.rows(), 1);
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
  Window &window = Window::get_instance();
  float asepect_ratio = float(window.width) / float(window.height);
  RenderFunc render_func = [&points, asepect_ratio](Scene &scene) {
    use_program(points.program);
    set_uniform_mat4(points.program, "projection", scene.camera.projection);
    set_uniform_RGB(points.program, "color", points.color);
    set_uniform_float(points.program, "pointSize", points.point_size / 200.0f);
    set_uniform_float(points.program, "aspectRatio", asepect_ratio);
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

Lines create_lines() {
  return Lines{
      0,
      create_vao(),    //
      create_vbo(),    //
      create_vbo(),    //
      create_program(  //
          create_shader(source::line_shader.vertex, GL_VERTEX_SHADER),
          create_shader(source::line_shader.fragment, GL_FRAGMENT_SHADER),
          create_shader(source::line_shader.geometry, GL_GEOMETRY_SHADER)),
      RGB(255, 0, 0),  //
      1.0f,
      true,
      1.0f,
      GL_LINE_STRIP};
}

void set_lines_data(Lines &lines, const MatxXf &points_data,
                    const MatxXf &per_line_color) {
  lines.n_points = points_data.rows();

  use_program(lines.program);

  bind_vao(lines.vertex_array);
  bind_vbo(lines.vertex_buffer);
  if (points_data.cols() == 2) {
    MatxXf points_data_3d(points_data.rows(), 3);
    points_data_3d << points_data, MatxXf::Zero(points_data.rows(), 1);
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
  Window &window = Window::get_instance();
  float asepect_ratio = float(window.width) / float(window.height);
  RenderFunc render_func = [&lines, asepect_ratio](Scene &scene) {
    use_program(lines.program);
    set_uniform_mat4(lines.program, "projection", scene.camera.projection);
    set_uniform_RGB(lines.program, "color", lines.color);
    set_uniform_float(lines.program, "alpha", lines.alpha);
    set_uniform_float(lines.program, "lineWidth", lines.width / 200.0f);
    set_uniform_float(lines.program, "aspectRatio", asepect_ratio);
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

Edges create_edges() {
  return Edges{
      0,
      create_vao(),    //
      create_vbo(),    //
      create_ebo(),    //
      create_vbo(),    //
      create_program(  //
          create_shader(source::line_shader.vertex, GL_VERTEX_SHADER),
          create_shader(source::line_shader.fragment, GL_FRAGMENT_SHADER),
          create_shader(source::line_shader.geometry, GL_GEOMETRY_SHADER)),
      RGB(255, 0, 0),  //
      1.0f,
      true,
      1.0f,
      GL_LINES};
}

void set_edges_data(Edges &edges, const MatxXf &points_data,
                    const Matx2i &indices, const MatxXf &per_line_color) {
  edges.n_edges = indices.rows();

  use_program(edges.program);

  bind_vao(edges.vertex_array);
  bind_vbo(edges.vertex_buffer);
  if (points_data.cols() == 2) {
    MatxXf points_data_3d(points_data.rows(), 3);
    points_data_3d << points_data, MatxXf::Zero(points_data.rows(), 1);
    set_vbo_dynamic_data(points_data_3d.data(),
                         points_data_3d.size() * sizeof(float));
  } else
    set_vbo_dynamic_data(points_data.data(),
                         points_data.size() * sizeof(float));
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  unbind_vbo();

  bind_ebo(edges.index_buffer);
  set_ebo_static_data(indices.data(), indices.size() * sizeof(int));

  if (per_line_color.rows() > 0) {
    bind_vbo(edges.color_buffer);
    set_vbo_dynamic_data(per_line_color.data(),
                         per_line_color.size() * sizeof(float));
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(1);
    unbind_vbo();
    edges.uniform_color = false;
  } else {
    edges.uniform_color = true;
  }
  unbind_vao();
  unuse_program();
}

RenderFunc get_render_func(Edges &edges) {
  Window &window = Window::get_instance();
  float asepect_ratio = float(window.width) / float(window.height);
  RenderFunc render_func = [&edges, asepect_ratio](Scene &scene) {
    use_program(edges.program);
    set_uniform_mat4(edges.program, "projection", scene.camera.projection);
    set_uniform_RGB(edges.program, "color", edges.color);
    set_uniform_float(edges.program, "alpha", edges.alpha);
    set_uniform_float(edges.program, "lineWidth", edges.width / 200.0f);
    set_uniform_float(edges.program, "aspectRatio", asepect_ratio);
    if (edges.uniform_color)
      set_uniform_float(edges.program, "choice", 1.0f);
    else
      set_uniform_float(edges.program, "choice", 0.0f);
    bind_vao(edges.vertex_array);
    glDrawElements(edges.mode, edges.n_edges * 2, GL_UNSIGNED_INT, 0);
    unbind_vao();
    unuse_program();
  };
  return render_func;
}

Lines create_deriv_lines() { return create_lines(); }

void set_deriv_lines_data(Lines &lines, const MatxXf &points_data,
                          const MatxXf &deriv_data, const float deriv_scale,
                          const MatxXf &per_line_color) {
  MatxXf new_points = points_data + deriv_data * deriv_scale;
  MatxXf points_pair(points_data.rows() * 2, points_data.cols());
  for (int i = 0; i < points_data.rows(); i++) {
    points_pair.row(2 * i) = points_data.row(i);
    points_pair.row(2 * i + 1) = new_points.row(i);
  }
  set_lines_data(lines, points_pair, per_line_color);
  lines.mode = GL_LINES;
}

Edges create_box_edges() { return create_edges(); }

void set_box_edges_data(Edges &edges, const Box3d &box) {
  MatxXf points(8, 3);
  Matx2f b = box.bound.cast<float>();
  points << b(0, 0), b(1, 0), b(2, 0),  //
      b(0, 0), b(1, 0), b(2, 1),        //
      b(0, 0), b(1, 1), b(2, 0),        //
      b(0, 0), b(1, 1), b(2, 1),        //
      b(0, 1), b(1, 0), b(2, 0),        //
      b(0, 1), b(1, 0), b(2, 1),        //
      b(0, 1), b(1, 1), b(2, 0),        //
      b(0, 1), b(1, 1), b(2, 1);
  Matx2i indices(12, 2);
  indices << 0, 1,  //
      0, 2,         //
      0, 4,         //
      1, 3,         //
      1, 5,         //
      2, 3,         //
      2, 6,         //
      3, 7,         //
      4, 5,         //
      4, 6,         //
      5, 7,         //
      6, 7;
  set_edges_data(edges, points, indices, MatxXf());
}

Triangles create_triangles() {
  return Triangles{
      0,
      create_vao(),
      create_vbo(),
      create_vbo(),
      create_program(
          create_shader(source::triangle_shader.vertex, GL_VERTEX_SHADER),
          create_shader(source::triangle_shader.fragment, GL_FRAGMENT_SHADER)),
      false,
      RGB(255, 255, 255),
      1.0f,
      GL_TRIANGLES,
      MatxXf()};
}

void set_triangles_data(Triangles &tris, const MatxXf &points_data,
                        const Matx3i &face_indices,
                        const MatxXf &per_face_color) {
  tris.n_faces = face_indices.rows();

  int n_faces = tris.n_faces;
  if (tris.vertex_buffer_data.rows() == 0) {
    tris.vertex_buffer_data.resize(3 * n_faces, 3);
    tris.color_buffer_data.resize(3 * n_faces, 3);
  }

  MatxXf points_data_f = points_data;

  // If points_data is Nx2, expand to Nx3:
  if (points_data_f.cols() == 2) {
    MatxXf expanded(points_data_f.rows(), 3);
    expanded.block(0, 0, points_data_f.rows(), 2) = points_data_f;
    expanded.col(2).setZero();
    points_data_f = expanded;
  }

  for (int f = 0; f < n_faces; f++) {
    int i0 = face_indices(f, 0);
    int i1 = face_indices(f, 1);
    int i2 = face_indices(f, 2);

    tris.vertex_buffer_data.row(3 * f + 0) = points_data_f.row(i0);
    tris.vertex_buffer_data.row(3 * f + 1) = points_data_f.row(i1);
    tris.vertex_buffer_data.row(3 * f + 2) = points_data_f.row(i2);

    // Face color
    // If per_face_color has a color per face, say (R,G,B) in each row
    Eigen::Vector3f face_col;
    if (per_face_color.rows() > 0) {
      face_col = per_face_color.row(f);
      tris.uniform_color = false;
    } else {
      // If no per-face color is provided, use uniform color
      face_col =
          Eigen::Vector3f(tris.color.x() / 255.0f, tris.color.y() / 255.0f,
                          tris.color.z() / 255.0f);
      tris.uniform_color = true;
    }
    tris.color_buffer_data.row(3 * f + 0) = face_col;
    tris.color_buffer_data.row(3 * f + 1) = face_col;
    tris.color_buffer_data.row(3 * f + 2) = face_col;
  }

  use_program(tris.program);
  bind_vao(tris.vertex_array);

  bind_vbo(tris.vertex_buffer);
  set_vbo_dynamic_data(tris.vertex_buffer_data.data(),
                       tris.vertex_buffer_data.size() * sizeof(float));
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  unbind_vbo();

  bind_vbo(tris.color_buffer);
  set_vbo_dynamic_data(tris.color_buffer_data.data(),
                       tris.color_buffer_data.size() * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  unbind_vbo();

  unbind_vao();
  unuse_program();
}

RenderFunc get_render_func(Triangles &faces) {
  Window &window = Window::get_instance();
  float aspect_ratio = float(window.width) / float(window.height);

  RenderFunc render_func = [&faces, aspect_ratio](Scene &scene) {
    use_program(faces.program);
    set_uniform_mat4(faces.program, "projection", scene.camera.projection);
    set_uniform_float(faces.program, "alpha", faces.alpha);
    bind_vao(faces.vertex_array);
    glDrawArrays(faces.mode, 0, faces.n_faces * 3);
    unbind_vao();
    unuse_program();
  };
  return render_func;
}

}  // namespace aphys
