#ifndef GL_OS_H_
#define GL_OS_H_

#include <string>

#include "ArmorerPhys/type.h"
#include "igl/readOBJ.h"
#include "igl/writeOBJ.h"
#include "igl/readMESH.h"
#include "igl/writeMESH.h"
#include "igl/readDMAT.h"
#include "igl/writeDMAT.h"

namespace aphys {

void writeOBJ(const std::string &filename, const MatxXd &V, const MatxXi &F);

}  // namespace aphys

#endif  // GL_OS_H_
