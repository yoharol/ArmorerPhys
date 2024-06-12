#include "ArmorerPhys/data/tetrahetralize.h"
#include "igl/copyleft/tetgen/tetrahedralize.h"

namespace aphys {

void tetgen(const MatxXd& verts, const Matx3i& faces, TetMesh& tm,
            const std::string& flags) {
  aphys::MatxXd TV;
  aphys::MatxXi TT, TF;
  igl::copyleft::tetgen::tetrahedralize(verts, faces, "pq1.414Y", TV, TT, TF);
  tm.verts = TV;
  tm.tets = TT;
  tm.faces = TF;
}

}  // namespace aphys
