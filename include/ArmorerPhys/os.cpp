#include "ArmorerPhys/os.h"

#include <string>

#include "ArmorerPhys/type.h"
#include "igl/writeOBJ.h"
#include "igl/readMESH.h"
#include "igl/writeMESH.h"
#include "igl/readDMAT.h"
#include "igl/writeDMAT.h"

namespace aphys {

void writeOBJ(const std::string &filename, const MatxXd &V, const MatxXi &F) {
  if (V.cols() == 2) {
    MatxXd V3 = V;
    V3.conservativeResize(V.rows(), 3);
    V3.col(2).setZero();
    igl::writeOBJ(filename, V3, F);
  } else if (V.cols() == 3) {
    igl::writeOBJ(filename, V, F);
  } else {
    throw std::runtime_error("writeOBJ: V must have 2 or 3 columns");
  }
}

}  // namespace aphys
