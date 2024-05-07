#ifndef GL_GEOM_H_
#define GL_GEOM_H_

#include "ArmorerPhys/type.h"

namespace aphys {

int find_nearest_point(const MatxXd &points, const Vecxd &point);

struct Box2d {
  Matx2d bound;
  Box2d(double l, double r, double y, double b) {
    bound.resize(2, 2);
    bound << l, r, y, b;
  }
};

struct Box3d {
  Matx2d bound;
  Box3d(double l, double r, double t, double b, double n, double f) {
    bound.resize(3, 2);
    bound << l, r, t, b, n, f;
  }
};

MatxXf get_normals(const MatxXf &vertices, const MatxXi &indices);

void create_rectangle(double l, double r, int hori_count, double b, double t,
                      int vert_count, MatxXd &vertices, Matx3i &indices);

void get_axis_value_indices(const double value, const int dim,
                            const MatxXd &vertices, Vecxi &axis_indices);

struct AffineControls {
  int n_control_points;
  int dim;
  MatxXd &points_ref_position;
  MatxXd T;
  Vecxd Tvec;
};

AffineControls create_affine_controls(MatxXd &points_ref_position);

void affine_organize_Tvec(AffineControls &controls);

void update_affine_rotation(AffineControls &controls, int idx, double angle);

void update_affine_rotation(AffineControls &controls, Vecxd &angles);

void update_affine_translation(AffineControls &controls, int idx,
                               Vecxd &translation);

void update_affine_from_world_translation(AffineControls &controls, int idx,
                                          Vecxd &translation);

void update_affine_from_world_translation(AffineControls &controls,
                                          MatxXd &translation);

void extract_edge(const Matx3i &faces, Matx2i &edge);

void compute_edge_length(const MatxXd &verts, const Matx2i &edge,
                         Vecxd &length);

}  // namespace aphys

#endif  // GL_GEOM_H_
