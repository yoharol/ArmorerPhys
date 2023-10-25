#ifndef GL_MATERIAL_H_
#define GL_MATERIAL_H_

#include <Eigen/Core>

#include "armgl_shader.h"
#include "armgl_buffer.h"

namespace agl {

struct Material {
  Eigen::Vector3f ambient;
  Eigen::Vector3f diffuse;
  Eigen::Vector3f specular;
  Shader shader;
  float shininess;
};

}  // namespace agl

#endif  // GL_MATERIAL_H_