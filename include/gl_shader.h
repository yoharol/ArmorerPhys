#ifndef GL_SHADER_H_
#define GL_SHADER_H_

#include "gl_type.h"
#include "gl_buffer.h"

namespace glrender {

struct ShaderSource {
  const char* vertex;
  const char* fragment;
};

namespace source {

const ShaderSource basic_shader = {
    R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    out vec4 vertexColor;
    uniform mat4 projection;
    void main() {
      gl_Position = projection * vec4(aPos, 1.0);
      gl_Position = gl_Position / gl_Position.w;
      vertexColor = vec4(aPos.xy, 0.0, 1.0);
      vertexColor = vertexColor * 0.5 + 0.5;
    }
  )",
    R"(
    #version 330 core
    out vec4 FragColor;
    in vec4 vertexColor;
    void main() {
      FragColor = vertexColor;
    }
  )"};

const ShaderSource basic_uv_shader = {
    R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec2 aTexCoord;

    out vec4 vertexColor;
    out vec2 TexCoord;
    
    uniform mat4 projection;
    
    void main() {
      gl_Position = projection * vec4(aPos, 1.0);
      gl_Position = gl_Position / gl_Position.w;
      vertexColor = vec4(aTexCoord, 0.0, 1.0);
      TexCoord = aTexCoord;
    }
  )",
    R"(
    #version 330 core
    out vec4 FragColor;
    in vec4 vertexColor;
    in vec2 TexCoord;

    uniform sampler2D texture1;

    void main() {
      FragColor = texture(texture1, TexCoord);
    }
  )"};

const ShaderSource basic_diffuse_shader = {
    R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aNormal;

    out vec3 FragPos;
    out vec3 Normal;

    uniform mat4 projection;

    void main()
    {
      gl_Position = projection * vec4(aPos, 1.0);
      gl_Position = gl_Position / gl_Position.w;
      FragPos = aPos;
      Normal = aNormal;
    }
  )",
    R"(
    #version 330 core
    out vec4 FragColor;

    in vec3 Normal;
    in vec3 FragPos;

    uniform float specularStrength;
    uniform vec3 viewPos;
    uniform vec3 lightPos;
    uniform vec3 lightColor;
    uniform vec3 diffuseColor;
    uniform vec3 ambientColor;
    uniform vec3 specularColor;

    void main()
    {
      vec3 ambient = ambientColor;
      vec3 norm = normalize(Normal);
      vec3 lightDir = normalize(lightPos - FragPos);
      float diff = max(dot(norm, lightDir), 0.0);
      vec3 diffuse = diff * lightColor;
      vec3 viewDir = normalize(viewPos - lightPos);
      vec3 reflectDir = reflect(-lightDir, norm);
      float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
      vec3 specular = specularStrength * spec * lightColor * specularColor;
      vec3 result = (ambient + diffuse + specular) * diffuseColor;
      FragColor = vec4(result, 1.0);
    }
  )"};

const ShaderSource textured_diffuse_shader = {
    R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aNormal;
    layout (location = 2) in vec2 aTexCoord;

    out vec3 FragPos;
    out vec3 Normal;
    out vec2 TexCoord;

    uniform mat4 projection;

    void main()
    {
      gl_Position = projection * vec4(aPos, 1.0);
      gl_Position = gl_Position / gl_Position.w;
      FragPos = aPos;
      Normal = aNormal;
      TexCoord = aTexCoord;
    }
  )",
    R"(
    #version 330 core
    out vec4 FragColor;

    in vec3 Normal;
    in vec3 FragPos;
    in vec2 TexCoord;
    
    uniform sampler2D texture1;

    uniform float specularStrength;
    uniform vec3 viewPos;
    uniform vec3 lightPos;
    uniform vec3 lightColor;
    uniform vec3 diffuseColor;
    uniform vec3 ambientColor;
    uniform vec3 specularColor;

    void main()
    {
      vec4 albedo = texture(texture1, TexCoord);
      vec3 ambient = ambientColor;
      vec3 norm = normalize(Normal);
      vec3 lightDir = normalize(lightPos - FragPos);
      float diff = max(dot(norm, lightDir), 0.0);
      vec3 diffuse = diff * lightColor;
      vec3 viewDir = normalize(viewPos - lightPos);
      vec3 reflectDir = reflect(-lightDir, norm);
      float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
      vec3 specular = specularStrength * spec * lightColor * specularColor;
      vec3 result = (ambient + diffuse + specular) * diffuseColor;
      FragColor = vec4(result, 1.0) * albedo;
    }
  )"};

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

}  // namespace glrender

#endif  // GL_SHADER_H_