#ifndef GL_MATERIAL_H_
#define GL_MATERIAL_H_

#include <Eigen/Core>

#include "ArmorerSim/render/shader.h"
#include "ArmorerSim/render/buffer.h"

namespace asim {

struct Material {
  Eigen::Vector3f ambient;
  Eigen::Vector3f diffuse;
  Eigen::Vector3f specular;
  Shader shader;
  float shininess;
};

}  // namespace asim

#endif  // GL_MATERIAL_H_