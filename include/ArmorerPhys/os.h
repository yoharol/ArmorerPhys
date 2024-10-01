#ifndef GL_OS_H_
#define GL_OS_H_

#include <string>

#include "ArmorerPhys/type.h"
#include "igl/readOBJ.h"

namespace aphys {

void writeOBJ(const std::string& filename, const MatxXd& V, const MatxXi& F);

template <typename DerivedF>
void readOBJ(const std::string& filename, MatxXd& V,
             Eigen::PlainObjectBase<DerivedF>& F, int dim) {
  if (dim == 2) {
    MatxXd V3;
    igl::readOBJ(filename, V3, F);
    V = V3.leftCols(2);
  } else if (dim == 3) {
    igl::readOBJ(filename, V, F);
  } else {
    throw std::runtime_error("readOBJ: dim must be 2 or 3");
  }
}

void export2DPolygonToSVG(const MatxXd& verts, const Matx2i& edge,
                          const Matx3i& faces, const Matx3f& colors,
                          const RGB& color, const std::string& filename,
                          double scale = 1.0);

void exportPolygonToPointBasedSVG(const MatxXd& V, const Matx2i& edge,
                                  const Matx3i& faces, const Matx3f& colors,
                                  const RGB& color, const std::string& filename,
                                  float radius, float edge_width,
                                  double stroke_width, double scale = 1.0,
                                  bool transparent = false);

}  // namespace aphys

#endif  // GL_OS_H_
