#ifndef ARMORER_SIM_COMMON_H_
#define ARMORER_SIM_COMMON_H_

#include "ArmorerPhys/type.h"

namespace aphys {

void extract_edge(const Matx3i& faces, Matx2i& edge);

void compute_edge_length(const MatxXf& verts, const Vecxi& edge, Vecxf& length);
}  // namespace aphys

#endif  // ARMORER_SIM_COMMON_H_