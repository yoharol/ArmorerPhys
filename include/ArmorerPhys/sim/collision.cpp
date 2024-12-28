#include "ArmorerPhys/sim/collision.h"

#include <ArmorerPhys/type.h>
#include <ArmorerPhys/glmath.h>

namespace aphys {

void collision2d(const Box2d& box, MatxXd& pos, double epsilon) {
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

void collision3d(const Box3d& box, MatxXd& pos, double epsilon) {
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

void collision3d(const Sphere3d& sphere, MatxXd& pos, double epsilon) {
  int N = pos.rows();
  for (int i = 0; i < N; i++) {
    Vec3d dir = pos.row(i).transpose() - sphere.center;
    double dist = dir.norm();
    if (dist < sphere.radius) {
      dir.normalize();
      pos.row(i) =
          sphere.center +
          dir * (sphere.radius + RandomEngine::getInstance()() * epsilon);
    }
  }
}

}  // namespace aphys
