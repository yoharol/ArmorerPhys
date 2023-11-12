#include "ArmorerPhys/sim/pbd/pbd_framework.h"

#include "ArmorerPhys/type.h"

namespace aphys {

void pbdPredict(MatxXf pos, MatxXf vel, MatxXf pos_cache, Vecxf external_force,
                float dt) {
  int N = pos.rows();
  for (int i = 0; i < N; i++) {
    pos_cache.row(i) = pos.row(i);
    pos.row(i) += dt * vel.row(i) + dt * dt * external_force.transpose();
  }
}

void pbdUpdateVelocity(MatxXf pos, MatxXf vel, MatxXf pos_cache, float dt) {
  int N = pos.rows();
  for (int i = 0; i < N; i++) {
    vel.row(i) = (pos.row(i) - pos_cache.row(i)) / dt;
  }
}
}  // namespace aphys