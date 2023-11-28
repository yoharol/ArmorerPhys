#include "ArmorerPhys/sim/pbd/pbd_framework.h"

#include "ArmorerPhys/type.h"

#include <cmath>

namespace aphys {

void PbdFramework::pbdPredict(MatxXd& pos, MatxXd& vel, MatxXd& pos_cache,
                              Vecxd& vert_invm, Vecxd& external_force,
                              double dt) {
  int N = pos.rows();
  for (int i = 0; i < N; i++) {
    pos_cache.row(i) = pos.row(i);
    pos.row(i) += dt * vel.row(i);
    if (vert_invm(i) > 0.0) pos.row(i) += dt * dt * external_force.transpose();
  }
}

void PbdFramework::pbdUpdateVelocity(MatxXd& pos, MatxXd& vel,
                                     MatxXd& pos_cache, double dt,
                                     double damping) {
  int N = pos.rows();
  for (int i = 0; i < N; i++) {
    vel.row(i) = (pos.row(i) - pos_cache.row(i)) * (exp(-damping * dt) / dt);
  }
}
}  // namespace aphys