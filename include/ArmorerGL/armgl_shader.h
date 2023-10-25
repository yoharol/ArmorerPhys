#ifndef GL_SHADER_H_
#define GL_SHADER_H_

#include "armgl_type.h"
#include "armgl_buffer.h"

namespace agl {

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

const ShaderSourceWithGeometry point_shader = {
    R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;

    uniform vec3 color;
    uniform float choice;
    uniform mat4 projection;

    out vec3 vertexColor;
    void main() {
      gl_Position = projection * vec4(aPos, 1.0);
      gl_Position = gl_Position / gl_Position.w;
      vertexColor = aColor * (1.0-choice) + choice * color;
    }
  )",
    R"(
    #version 330 core

    in vec3 fragColor;
    out vec4 FragColor;

    void main() {
      FragColor = vec4(fragColor, 1.0);
    }
  )",
    R"(
    #version 330 core
    
    layout (points) in;
    layout (triangle_strip, max_vertices = 48) out;

    uniform float pointSize;
    uniform float aspectRatio;

    in vec3 vertexColor[];
    out vec3 fragColor;

    void main(){
      float tri_num = 16.0;
      fragColor = vertexColor[0];
      for (int i=0; i<tri_num; i++){
        float angle = i * 2.0 * 3.14159265359 / tri_num;
        float angle_next = (i+1) * 2.0 * 3.14159265359 / tri_num;
        vec2 offset = vec2(cos(angle), sin(angle)) * pointSize / 2.0;
        vec2 offset_next = vec2(cos(angle_next), sin(angle_next)) * pointSize / 2.0;
        offset.x = offset.x / aspectRatio;
        offset_next.x = offset_next.x / aspectRatio;
        gl_Position = gl_in[0].gl_Position + vec4(offset, 0.0, 0.0);
        EmitVertex();
        gl_Position = gl_in[0].gl_Position + vec4(offset_next, 0.0, 0.0);
        EmitVertex();
        gl_Position = gl_in[0].gl_Position;
        EmitVertex();
      }
      // gl_Position = gl_in[0].gl_Position + vec4(pointSize/2.0, 0.0, 0.0, 0.0);
      // EmitVertex();
      EndPrimitive();
    }
  )"};

const ShaderSourceWithGeometry line_shader = {
    R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;

    uniform vec3 color;
    uniform float choice;
    uniform mat4 projection;

    out vec3 vertexColor;
    void main() {
      gl_Position = projection * vec4(aPos, 1.0);
      gl_Position = gl_Position / gl_Position.w;
      vertexColor = aColor * (1.0-choice) + choice * color;
    }
  )",
    R"(
    #version 330 core

    in vec3 fragColor;
    out vec4 FragColor;

    uniform float alpha;

    void main() {
      FragColor = vec4(fragColor, alpha);
    }
  )",
    R"(
    #version 330 core

    layout (lines) in;
    layout (triangle_strip, max_vertices = 4) out;

    uniform float lineWidth;
    uniform float aspectRatio;

    in vec3 vertexColor[];
    out vec3 fragColor;

    void main(){
      vec2 p1 = gl_in[0].gl_Position.xy;
      vec2 p2 = gl_in[1].gl_Position.xy;
      vec2 dir = normalize(p2 - p1);
      vec2 normal = vec2(-dir.y, dir.x);
      vec2 offset = normal * lineWidth / 2.0;
      offset.x = offset.x / aspectRatio;

      fragColor = vertexColor[0];
      gl_Position = vec4(p1 + offset, 0.0, 1.0);
      EmitVertex();
      gl_Position = vec4(p1 - offset, 0.0, 1.0);
      EmitVertex();
      fragColor = vertexColor[1];
      gl_Position = vec4(p2 + offset, 0.0, 1.0);
      EmitVertex();
      gl_Position = vec4(p2 - offset, 0.0, 1.0);
      EmitVertex();
      EndPrimitive();
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

const Light default_light = {
    {242, 242, 242},     // light color
    {9, 5, 88},          // ambient color
    {0.0f, 0.35f, 5.0f}  // light position
};

}  // namespace agl

#endif  // GL_SHADER_H_