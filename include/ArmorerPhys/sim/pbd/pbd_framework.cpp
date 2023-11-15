#include "ArmorerPhys/sim/pbd/pbd_framework.h"

#include "ArmorerPhys/type.h"

#include <cmath>

namespace aphys {

void PbdFramework::pbdPredict(MatxXf& pos, MatxXf& vel, MatxXf& pos_cache,
                              Vecxf& vert_invm, Vecxf& external_force,
                              float dt) {
  int N = pos.rows();
  for (int i = 0; i < N; i++) {
    pos_cache.row(i) = pos.row(i);
    pos.row(i) += dt * vel.row(i);
    if (vert_invm(i) > 0.0) pos.row(i) += dt * dt * external_force.transpose();
  }
}

void PbdFramework::pbdUpdateVelocity(MatxXf& pos, MatxXf& vel,
                                     MatxXf& pos_cache, float dt,
                                     float damping) {
  int N = pos.rows();
  for (int i = 0; i < N; i++) {
    vel.row(i) = (pos.row(i) - pos_cache.row(i)) * (exp(-damping * dt) / dt);
  }
}
}  // namespace aphys