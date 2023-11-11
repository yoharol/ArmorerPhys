#include "ArmorerPhys/render/buffer.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>

#include "ArmorerPhys/image.h"

namespace aphys {

Shader create_shader(const char* shader_source, GLenum shader_type) {
  Shader shader;
  shader.id = glCreateShader(shader_type);
  glShaderSource(shader.id, 1, &shader_source, NULL);
  glCompileShader(shader.id);
  // check for shader compile errors
  int success;
  char infoLog[512];
  glGetShaderiv(shader.id, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shader.id, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }
  return shader;
}

Program create_program(Shader vertex_shader, Shader fragment_shader) {
  Program program;
  program.id = glCreateProgram();
  glAttachShader(program.id, vertex_shader.id);
  glAttachShader(program.id, fragment_shader.id);
  glLinkProgram(program.id);
  // check for linking errors
  int success;
  char infoLog[512];
  glGetProgramiv(program.id, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(program.id, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
              << infoLog << std::endl;
  }
  return program;
}

Program create_program(Shader vertex_shader, Shader fragment_shader,
                       Shader geometry_shader) {
  Program program;
  program.id = glCreateProgram();
  glAttachShader(program.id, vertex_shader.id);
  glAttachShader(program.id, fragment_shader.id);
  glAttachShader(program.id, geometry_shader.id);
  glLinkProgram(program.id);
  // check for linking errors
  int success;
  char infoLog[512];
  glGetProgramiv(program.id, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(program.id, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
              << infoLog << std::endl;
  }
  return program;
}

}  // namespace aphys
