#include "ArmorerPhys/sim/collision.h"

#include <ArmorerPhys/type.h>
#include <ArmorerPhys/math.h>

namespace aphys {

void collision2d(const Box2d& box, Matx2f& pos, float epsilon) {
  int N = pos.rows();
  for (int i = 0; i < N; i++) {
    if (pos(i, 0) < box.bound(0, 0))
      pos(i, 0) = box.bound(0, 0) + epsilon * RandomEngine::getInstance()();
    if (pos(i, 0) > box.bound(0, 1))
      pos(i, 0) = box.bound(0, 1) - epsilon * RandomEngine::getInstance()();
    if (pos(i, 1) < box.bound(1, 0))
      pos(i, 1) = box.bound(1, 0) + epsilon * RandomEngine::getInstance()();
    if (pos(i, 1) > box.bound(1, 1))
      pos(i, 1) = box.bound(1, 1) - epsilon * RandomEngine::getInstance()();
  }
}

void collision3d(const Box3d& box, Matx3f& pos, float epsilon) {
  int N = pos.rows();
  for (int i = 0; i < N; i++) {
    if (pos(i, 0) < box.bound(0, 0))
      pos(i, 0) = box.bound(0, 0) + epsilon * RandomEngine::getInstance()();
    if (pos(i, 0) > box.bound(0, 1))
      pos(i, 0) = box.bound(0, 1) - epsilon * RandomEngine::getInstance()();
    if (pos(i, 1) < box.bound(1, 0))
      pos(i, 1) = box.bound(1, 0) + epsilon * RandomEngine::getInstance()();
    if (pos(i, 1) > box.bound(1, 1))
      pos(i, 1) = box.bound(1, 1) - epsilon * RandomEngine::getInstance()();
    if (pos(i, 2) < box.bound(2, 0))
      pos(i, 2) = box.bound(2, 0) + epsilon * RandomEngine::getInstance()();
    if (pos(i, 2) > box.bound(2, 1))
      pos(i, 2) = box.bound(2, 1) - epsilon * RandomEngine::getInstance()();
  }
}

}  // namespace aphys
