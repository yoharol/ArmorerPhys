#ifndef GL_OS_H_
#define GL_OS_H_

#include <string>

#include "ArmorerPhys/type.h"
#include "igl/readOBJ.h"

namespace aphys {

void writeOBJ(const std::string &filename, const MatxXd &V, const MatxXi &F);

template <typename DerivedF>
void readOBJ(const std::string &filename, MatxXd &V,
             Eigen::PlainObjectBase<DerivedF> &F, int dim) {
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

}  // namespace aphys

#endif  // GL_OS_H_
