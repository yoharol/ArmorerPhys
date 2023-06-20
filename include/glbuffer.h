#ifndef BUFFER_DATA_H_
#define BUFFER_DATA_H_

#include "glad/glad.h"
#include "GLFW/glfw3.h"

namespace glrender {

struct VAO {
  unsigned int id;
};

VAO create_vao() {
  VAO vao;
  glGenVertexArrays(1, &vao.id);
  return vao;
}

void bind_vao(VAO vao) { glBindVertexArray(vao.id); }

void unbind_vao() { glBindVertexArray(0); }

void delete_vao(VAO vao) { glDeleteVertexArrays(1, &vao.id); }

struct VBO {
  unsigned int id;
};

VBO create_vbo() {
  VBO vbo;
  glGenBuffers(1, &vbo.id);
  return vbo;
}

void bind_vbo(VBO vbo) { glBindBuffer(GL_ARRAY_BUFFER, vbo.id); }

void unbind_vbo() { glBindBuffer(GL_ARRAY_BUFFER, 0); }

void delete_vbo(VBO vbo) { glDeleteBuffers(1, &vbo.id); }

template <typename T>
void set_vbo_static_data(T* vertices, size_t size) {
  glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);
}

template <typename T>
void set_vbo_dynamic_data(T* vertices, size_t size) {
  glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_DYNAMIC_DRAW);
}

struct EBO {
  unsigned int id;
};

EBO create_ebo() {
  EBO ebo;
  glGenBuffers(1, &ebo.id);
  return ebo;
}

void bind_ebo(EBO ebo) { glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo.id); }

void unbind_ebo() { glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); }

void delete_ebo(EBO ebo) { glDeleteBuffers(1, &ebo.id); }

struct Shader {
  unsigned int id;
};

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

void delete_shader(Shader shader) { glDeleteShader(shader.id); }

struct Program {
  unsigned int id;
};

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

void use_program(Program program) { glUseProgram(program.id); }

void unuse_program() { glUseProgram(0); }

void delete_program(Program program) { glDeleteProgram(program.id); }

struct Texture {
  unsigned int id;
};

Texture create_texture() {
  Texture texture;
  glGenTextures(1, &texture.id);
  return texture;
}

void bind_texture(Texture texture) { glBindTexture(GL_TEXTURE_2D, texture.id); }

void unbind_texture() { glBindTexture(GL_TEXTURE_2D, 0); }

void set_texture_parameter(Texture texture, GLenum pname, GLint param) {
  glTexParameteri(GL_TEXTURE_2D, pname, param);
}

void set_texture_image(Texture texture, GLint level, GLint internalformat,
                       GLsizei width, GLsizei height, GLint border,
                       GLenum format, GLenum type, const void* pixels) {
  glTexImage2D(GL_TEXTURE_2D, level, internalformat, width, height, border,
               format, type, pixels);
}

void delete_texture(Texture texture) { glDeleteTextures(1, &texture.id); }

}  // namespace glrender

#endif  // BUFFER_DATA_H_