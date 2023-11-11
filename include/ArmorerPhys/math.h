#ifndef GL_MATH_H_
#define GL_MATH_H_

#include "ArmorerPhys/type.h"

namespace aphys {

Vec3f heat_rgb(float value, float minv, float maxv);

void Mat3fToList3f(const Matx3f& mat, Listx3f& vec);

void List3fToMat3f(const Listx3f& vec, Matx3f& mat);

void List3fToList2f(const Listx3f& vec3, Listx2f& vec2);

void List2fToList3f(const Listx2f& vec2, Listx3f& vec3);

void Mat2fToList3f(const Matx2f& mat, Listx3f& vec);

};  // namespace aphys

#endif  // GL_MATH_H_