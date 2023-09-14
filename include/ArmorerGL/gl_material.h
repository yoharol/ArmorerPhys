#ifndef GL_MATERIAL_H_
#define GL_MATERIAL_H_

#include <Eigen/Core>

#include "gl_shader.h"
#include "gl_buffer.h"

namespace glrender {

struct Material {
  Eigen::Vector3f ambient;
  Eigen::Vector3f diffuse;
  Eigen::Vector3f specular;
  Shader shader;
  float shininess;
};

}  // namespace glrender

#endif  // GL_MATERIAL_H_