#include "ArmorerPhys/render/mathplot2d.h"

#include <Eigen/Geometry>
#include <vector>

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/render/objs.h"

namespace aphys {

Lines create_ruler2d(float x1, float y1, float x2, float y2, RGB color) {
  Lines lines = create_lines();
  Matx2f verts(2, 2);
  verts << x1, y1, x2, y2;
  set_lines_data(lines, verts, MatxXf());
  lines.mode = GL_LINES;
  lines.color = color;
  return lines;
}

void set_ruler2d_data(Lines &lines, float x1, float y1, float x2, float y2) {
  Matx2f verts(2, 2);
  verts << x1, y1, x2, y2;
  set_lines_data(lines, verts, MatxXf());
}

void set_ruler2d_data(Lines &lines, Vec2f p1, Vec2f p2, float length) {
  Vec2f v = p2 - p1;
  v.normalize();
  Vec2f v1 = p1 + v * length;
  Vec2f v2 = p1 + v * (-length);
  set_ruler2d_data(lines, v1(0), v1(1), v2(0), v2(1));
}

Lines create_axis2d(float x_min, float x_max, float y_min, float y_max,
                    RGB color) {
  Lines lines = create_lines();
  Matx2f verts(4, 2);
  verts << x_min, 0.f,  //
      x_max, 0.f,       //
      0.f, y_min,       //
      0.f, y_max;       //
  set_lines_data(lines, verts, MatxXf());
  lines.color = color;
  lines.mode = GL_LINES;
  return lines;
}

Lines create_grid_axis2d(float x_min, float x_max, float y_min, float y_max,
                         int gridx, int gridy, RGB color) {
  Lines lines = create_lines();
  Matx2f verts(2 * (gridx + gridy + 2), 2);
  for (int i = 0; i <= gridx; i++) {
    float x = (x_max - x_min) * i / gridx + x_min;
    verts.row(2 * i) << x, y_min;
    verts.row(2 * i + 1) << x, y_max;
  }
  for (int i = 0; i <= gridy; i++) {
    float y = (y_max - y_min) * i / gridy + y_min;
    verts.row(2 * (gridx + 1) + 2 * i) << x_min, y;
    verts.row(2 * (gridx + 1) + 2 * i + 1) << x_max, y;
  }
  set_lines_data(lines, verts, MatxXf());
  lines.color = color;
  lines.mode = GL_LINES;
  return lines;
}

ControlPoint create_control_point() {
  ControlPoint control_point;
  control_point.scale = 0.1f;
  control_point.points = create_points();
  control_point.directions = create_edges();
  return control_point;
}

void set_control_point_data(ControlPoint &control_points,
                            AffineControls &control,
                            const MatxXf &per_point_color) {
  int n = control.n_control_points;
  int dim = control.dim;
  MatxXd display_points(n, dim);
  MatxXd edge_points(n * dim * 2, dim);
  Matx2i edge_indices(n * dim, 2);
  MatxXf edge_colors(n * dim * 2, 3);
  MatxXf per_direction_color(dim, 3);
  per_direction_color.block(0, 0, dim, dim) = MatxXf::Identity(dim, dim);

  for (int i = 0; i < n; i++) {
    MatxXd p(dim + 1, dim + 1);
    p.col(dim).setOnes();
    p.block(0, 0, 1, dim) = control.points_ref_position.row(i);
    MatxXd I = MatxXd::Identity(dim, dim);
    for (int j = 0; j < dim; j++) {
      p.block(j + 1, 0, 1, dim) =
          control.points_ref_position.row(i) +
          I.row(j) * static_cast<double>(control_points.scale);
    }
    MatxXd T(dim, dim + 1);
    T = control.T.block(i * dim, 0, dim, dim + 1);
    p = p * T.transpose();
    display_points.row(i) = p.row(0);

    for (int j = 0; j < dim; j++) {
      edge_points.row(i * dim * 2 + j * 2) = p.row(0);
      edge_colors.row(i * dim * 2 + j * 2) = per_direction_color.row(j);
      edge_points.row(i * dim * 2 + j * 2 + 1) = p.row(j + 1);
      edge_colors.row(i * dim * 2 + j * 2 + 1) = per_direction_color.row(j);

      edge_indices(i * dim + j, 0) = i * dim * 2 + j * 2;
      edge_indices(i * dim + j, 1) = i * dim * 2 + j * 2 + 1;
    }
  }

  set_points_data(control_points.points, display_points.cast<float>(),
                  per_point_color);
  set_edges_data(control_points.directions, edge_points.cast<float>(),
                 edge_indices, edge_colors);
}

RenderFunc get_render_func(ControlPoint &control_points) {
  RenderFunc render_func = [&control_points](Scene scene) {
    get_render_func(control_points.points)(scene);
    get_render_func(control_points.directions)(scene);
  };
  return render_func;
}

}  // namespace aphys
