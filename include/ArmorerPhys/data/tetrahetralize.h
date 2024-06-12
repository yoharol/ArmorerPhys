#ifndef ARMORPHYS_DATA_TETRAHEDRALIZE_H_
#define ARMORPHYS_DATA_TETRAHEDRALIZE_H_

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/tet.h"
#include "ArmorerPhys/data/tet_bind.h"

namespace aphys {

void tetgen(const MatxXd& verts, const Matx3i& faces, TetMesh& tm,
            const std::string& flags = "pq1.414Y");

}  // namespace aphys

#endif  // ARMORPHYS_DATA_TETRAHEDRALIZE_H_
