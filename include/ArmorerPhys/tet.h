#ifndef GL_TET_H_
#define GL_TET_H_

#include "ArmorerPhys/type.h"

namespace aphys {

struct TetMesh {
  MatxXd verts;
  Matx4i tets;
  Matx3i faces;
};

void create_rectangular_prism(double l, double r, int x_count, double b,
                              double t, int y_count, double n, double f,
                              int z_count, MatxXd& verts, Matx4i& tet_indices);

void extract_surface_from_tets(const Vecxd& verts, double l, double r, double b,
                               double t, double n, double f, Matx4i& tets,
                               Matx3i& faces);
void extract_surface_from_tets(const int n_verts, const Matx4i& tets,
                               Matx3i& faces);

struct VisualTetMesh {
  MatxXd visual_verts;
  Matx3i visual_faces;
  Matx3f visual_colors;
};

void extract_visual_tets_surfaces(const Matx4i& tets, const MatxXd& verts,
                                  Matx3i& visual_faces);

void construct_visual_tets(MatxXd& visual_verts, const MatxXd& verts,
                           const Matx4i& tets,
                           const double shrink_ratio = 0.9f);

// void fix_visual_tets_orientation(int n_tets, const MatxXd& visual_verts,
//                                  Matx3i& visual_faces);

void construct_visual_tets_color(Matx3f& visual_verts_color,
                                 const Matx3f& verts_color, const Matx4i& tets);

}  // namespace aphys

#endif  // GL_TET_H_
