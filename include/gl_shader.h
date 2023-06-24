#ifndef GL_SHADER_H_
#define GL_SHADER_H_

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

    uniform vec3 lightPos;
    uniform vec3 lightColor;
    uniform vec3 objectColor;
    uniform vec3 ambientColor;

    void main()
    {
      vec3 ambient = ambientColor;
      vec3 norm = normalize(Normal);
      vec3 lightDir = normalize(lightPos - FragPos);
      float diff = max(dot(norm, lightDir), 0.0);
      vec3 diffuse = diff * lightColor;
      vec3 result = (ambient + diffuse) * objectColor;
      FragColor = vec4(result, 1.0);
    }
  )"};

}  // namespace source

}  // namespace glrender

#endif  // GL_SHADER_H_