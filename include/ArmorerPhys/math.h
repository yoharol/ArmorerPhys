#ifndef GL_MATH_H_
#define GL_MATH_H_

#include <random>

#include "ArmorerPhys/type.h"

namespace aphys {

struct RandomEngine {
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<float> dis;

  float operator()() { return dis(gen); }
  float operator()(float minv, float maxv) {
    return dis(gen) * (maxv - minv) + minv;
  }

  static RandomEngine& getInstance() {
    static RandomEngine engine;
    return engine;
  }
  RandomEngine(RandomEngine const&) = delete;
  RandomEngine(RandomEngine&&) = delete;

 private:
  RandomEngine() : gen(rd()), dis(0.0, 1.0) {}
};

Vec3f heat_rgb(float value, float minv, float maxv);

void Mat3fToList3f(const Matx3f& mat, Listx3f& vec);

void List3fToMat3f(const Listx3f& vec, Matx3f& mat);

void List3fToList2f(const Listx3f& vec3, Listx2f& vec2);

void List2fToList3f(const Listx2f& vec2, Listx3f& vec3);

void Mat2fToList3f(const Matx2f& mat, Listx3f& vec);

};  // namespace aphys

#endif  // GL_MATH_H_