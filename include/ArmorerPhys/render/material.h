#ifndef GL_MATERIAL_H_
#define GL_MATERIAL_H_

#include <Eigen/Core>
#include "ArmorerPhys/render/shader.h"
#include "ArmorerPhys/render/buffer.h"
#include "ArmorerPhys/type.h"

namespace aphys {

struct Material {
  Vec3f ambient;
  Vec3f diffuse;
  Vec3f specular;
  Shader shader;
  float shininess;
};

}  // namespace aphys

#endif  // GL_MATERIAL_H_