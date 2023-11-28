#ifndef ARMORER_SIM_COLLISION_H_
#define ARMORER_SIM_COLLISION_H_

#include "ArmorerPhys/geom.h"

namespace aphys {

void collision2d(const Box2d& box, MatxXd& pos, double epsilon = 1e-5);

void collision3d(const Box3d& box, MatxXd& pos, double epsilon = 1e-5);

}  // namespace aphys

#endif  // ARMORER_SIM_COLLISION_H_