#ifndef GL_OS_H_
#define GL_OS_H_

#include <string>
#include <vector>

#include <igl/writeOBJ.h>

namespace aphys {

inline void writeOBJ(const std::string &filename, const MatxXf &V,
                     const MatxXi &F) {
  if (V.cols() == 2) {
    MatxXf V3(V.rows(), 3);
    V3 << V, MatxXf::Zero(V.rows(), 1);
    igl::writeOBJ(filename, V3, F);
  } else if (V.cols() == 3) {
    igl::writeOBJ(filename, V, F);
  } else {
    throw std::runtime_error("writeOBJ: V must have 2 or 3 columns");
  }
  igl::writeOBJ(filename, V, F);
}
}  // namespace aphys

#endif  // GL_OS_H_