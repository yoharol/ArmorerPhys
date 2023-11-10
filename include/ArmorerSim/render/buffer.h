#ifndef BUFFER_DATA_H_
#define BUFFER_DATA_H_

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>

#include "ArmorerSim/image.h"

namespace asim {

struct VAO {
  unsigned int id;
};

inline VAO create_vao() {
  VAO vao;
  glGenVertexArrays(1, &vao.id);
  return vao;
}

inline void bind_vao(VAO vao) { glBindVertexArray(vao.id); }

inline void unbind_vao() { glBindVertexArray(0); }

inline void delete_vao(VAO vao) { glDeleteVertexArrays(1, &vao.id); }

struct VBO {
  unsigned int id;
};

inline VBO create_vbo() {
  VBO vbo;
  glGenBuffers(1, &vbo.id);
  return vbo;
}

inline void bind_vbo(VBO vbo) { glBindBuffer(GL_ARRAY_BUFFER, vbo.id); }

inline void unbind_vbo() { glBindBuffer(GL_ARRAY_BUFFER, 0); }

inline void delete_vbo(VBO vbo) { glDeleteBuffers(1, &vbo.id); }

template <typename T>
inline void set_vbo_static_data(T* vertices, size_t size) {
  glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);
}

template <typename T>
inline void set_vbo_dynamic_data(T* vertices, size_t size) {
  glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_DYNAMIC_DRAW);
}

struct EBO {
  unsigned int id;
};

inline EBO create_ebo() {
  EBO ebo;
  glGenBuffers(1, &ebo.id);
  return ebo;
}

inline void bind_ebo(EBO ebo) { glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo.id); }

inline void unbind_ebo() { glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); }

inline void delete_ebo(EBO ebo) { glDeleteBuffers(1, &ebo.id); }

template <typename T>
inline void set_ebo_static_data(T* indices, size_t size) {
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, size, indices, GL_STATIC_DRAW);
}

template <typename T>
inline void set_ebo_dynamic_data(T* indices, size_t size) {
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, size, indices, GL_DYNAMIC_DRAW);
}

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

inline void delete_shader(Shader shader) { glDeleteShader(shader.id); }

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

inline void use_program(Program program) { glUseProgram(program.id); }

inline void unuse_program() { glUseProgram(0); }

inline void delete_program(Program program) { glDeleteProgram(program.id); }

inline void set_uniform_bool(Program program, const char* name, bool value) {
  glUniform1i(glGetUniformLocation(program.id, name), (int)value);
}

inline void set_uniform_int(Program program, const char* name, int value) {
  glUniform1i(glGetUniformLocation(program.id, name), value);
}

inline void set_uniform_float(Program program, const char* name, float value) {
  glUniform1f(glGetUniformLocation(program.id, name), value);
}

inline void set_uniform_float2(Program program, const char* name, float value1,
                               float value2) {
  glUniform2f(glGetUniformLocation(program.id, name), value1, value2);
}

inline void set_uniform_float3(Program program, const char* name, float value1,
                               float value2, float value3) {
  glUniform3f(glGetUniformLocation(program.id, name), value1, value2, value3);
}

inline void set_uniform_float3(Program program, const char* name, Vec3f value) {
  glUniform3f(glGetUniformLocation(program.id, name), value.x(), value.y(),
              value.z());
}

inline void set_uniform_RGB(Program program, const char* name, RGB value) {
  glUniform3f(glGetUniformLocation(program.id, name), float(value.x()) / 255.0f,
              float(value.y()) / 255.0f, float(value.z()) / 255.0f);
}

inline void set_uniform_float4(Program program, const char* name, float value1,
                               float value2, float value3, float value4) {
  glUniform4f(glGetUniformLocation(program.id, name), value1, value2, value3,
              value4);
}

inline void set_uniform_mat4(Program program, const char* name, Mat4f value) {
  glUniformMatrix4fv(glGetUniformLocation(program.id, name), 1, GL_FALSE,
                     value.data());
}

struct Texture {
  unsigned int id;
};

inline Texture create_texture() {
  Texture texture;
  glGenTextures(1, &texture.id);
  return texture;
}

inline void bind_texture(Texture texture) {
  glBindTexture(GL_TEXTURE_2D, texture.id);
}

inline void unbind_texture() { glBindTexture(GL_TEXTURE_2D, 0); }

inline void set_texture_parameter(GLenum pname, GLint param) {
  glTexParameteri(GL_TEXTURE_2D, pname, param);
}

inline void set_texture_wrap(GLint param) {
  set_texture_parameter(GL_TEXTURE_WRAP_S, param);
  set_texture_parameter(GL_TEXTURE_WRAP_T, param);
}

inline void set_texture_filter(GLint param_min, GLint param_mag) {
  set_texture_parameter(GL_TEXTURE_MIN_FILTER, param_min);
  set_texture_parameter(GL_TEXTURE_MAG_FILTER, param_mag);
}

inline void set_texture_image(GLint level, Image image, GLint internalformat,
                              GLenum format, GLenum type) {
  glTexImage2D(GL_TEXTURE_2D, level, internalformat, image.width, image.height,
               0, format, type, image.data);
}

inline void generate_texture_mipmap() { glGenerateMipmap(GL_TEXTURE_2D); }

inline void delete_texture(Texture texture) {
  glDeleteTextures(1, &texture.id);
}

}  // namespace asim

#endif  // BUFFER_DATA_H_