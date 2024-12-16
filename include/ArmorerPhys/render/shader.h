#ifndef GL_SHADER_H_
#define GL_SHADER_H_

#include "ArmorerPhys/type.h"

namespace aphys {

struct ShaderSource {
  const char* vertex;
  const char* fragment;
};

struct ShaderSourceWithGeometry {
  const char* vertex;
  const char* fragment;
  const char* geometry;
};

namespace source {

extern const ShaderSource basic_shader;

extern const ShaderSource basic_uv_shader;

extern const ShaderSourceWithGeometry point_shader;

extern const ShaderSourceWithGeometry line_shader;

extern const ShaderSource triangle_shader;

extern const ShaderSource basic_diffuse_shader;

extern const ShaderSource color_diffuse_shader;

extern const ShaderSource textured_diffuse_shader;

}  // namespace source

struct DiffuseMaterial {
  RGB diffuse_color;
  RGB specular_color;
  float specular_strength;
};

struct Light {
  RGB light_color;
  RGB ambient_color;
  Vec3f position;
};

extern const Light default_light;

}  // namespace aphys

#endif  // GL_SHADER_H_
