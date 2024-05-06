#ifndef GL_TET_H_
#define GL_TET_H_

#include "ArmorerPhys/type.h"

namespace aphys {

void create_rectangular_prism(double l, double r, int x_count, double b,
                              double t, int y_count, double n, double f,
                              int z_count, MatxXd& verts, Matx4i& tet_indices);

void extract_surface_from_tets(const Vecxd& verts, double l, double r, double b,
                               double t, double n, double f, Matx4i& tets,
                               Matx3i& faces);
void extract_surface_from_tets(const int n_verts, const Matx4i& tets,
                               Matx3i& faces);

}  // namespace aphys

#endif  // GL_TET_H_
