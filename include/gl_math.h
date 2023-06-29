#ifndef GL_MATH_H_
#define GL_MATH_H_

#include <algorithm>
#include <Eigen/Core>

namespace glrender {

Vec3f heat_rgb(float value, float minv, float maxv) {
  float ratio = 2 * (value - minv) / (maxv - minv);
  float b = std::max(0.0f, 1 - ratio);
  float r = std::max(0.0f, ratio - 1);
  float g = 1 - b - r;
  return Vec3f(r, g, b);
}

};  // namespace glrender

#endif  // GL_MATH_H_