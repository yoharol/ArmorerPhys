#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "gl_render.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  glrender::init_glfw();
  GLFWwindow *window = glrender::create_window(SCR_WIDTH, SCR_HEIGHT,
                                               "Example1: Textured Triangles");
  glrender::init_glad();

  // build and compile our shader program

  glrender::Shader vertex_shader = glrender::create_shader(
      glrender::source::basic_uv_shader.vertex, GL_VERTEX_SHADER);
  glrender::Shader fragment_shader = glrender::create_shader(
      glrender::source::basic_uv_shader.fragment, GL_FRAGMENT_SHADER);

  glrender::Program program =
      glrender::create_program(vertex_shader, fragment_shader);

  // link shaders
  glrender::delete_shader(vertex_shader);
  glrender::delete_shader(fragment_shader);

  // set up vertex data (and buffer(s)) and configure vertex attributes
  // ------------------------------------------------------------------
  Eigen::VectorXf vertices(12);
  vertices << -0.5f, -0.5f, 0.0f,  // left
      0.5f, -0.5f, 0.0f,           // right
      -0.5f, 0.5f, 0.0f,           // top left
      0.5f, 0.5f, 0.0f;            // top right
  int n_vertices = vertices.size() / 3;
  Eigen::VectorXf texCoords(8);
  texCoords << 0.0f, 0.0f,  // lower-left corner
      1.0f, 0.0f,           // lower-right corner
      0.0f, 1.0f,           // top-left corner
      1.0f, 1.0f;           // top-right corner
  Eigen::VectorXi indices(6);
  indices << 0, 1, 2,  // first triangle
      1, 3, 2;         // second triangle
  int n_faces = indices.size() / 3;

  glrender::VAO vao = glrender::create_vao();
  glrender::bind_vao(vao);

  glrender::EBO index_buffer = glrender::create_ebo();
  glrender::bind_ebo(index_buffer);
  glrender::set_ebo_static_data(indices.data(), indices.size() * sizeof(int));

  glrender::VBO vertex_buffer = glrender::create_vbo();
  glrender::bind_vbo(vertex_buffer);
  glrender::set_vbo_static_data(vertices.data(),
                                vertices.size() * sizeof(float));

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glrender::unbind_vbo();

  glrender::VBO uv_buffer = glrender::create_vbo();
  glrender::bind_vbo(uv_buffer);
  glrender::set_vbo_static_data(texCoords.data(),
                                texCoords.size() * sizeof(float));
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  glrender::unbind_vbo();

  glrender::unbind_vao();

  glrender::Texture tex = glrender::create_texture();
  glrender::bind_texture(tex);
  glrender::set_texture_wrap(GL_CLAMP);
  glrender::set_texture_filter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
  glrender::Image img =
      glrender::load_image(std::string(ASSETS_PATH) + "/" + "awesomeface.png");
  glrender::set_texture_image(0, img, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE);
  glrender::generate_texture_mipmap();
  glrender::free_image(img);
  glrender::unbind_texture();

  // set blend mode
  glrender::set_blend_transparent();
  glrender::set_wireframe_mode(false);

  glrender::Camera camera = glrender::create_camera(
      {0.0f, 0.0f, 3.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
      float(SCR_WIDTH) / float(SCR_HEIGHT));

  glrender::use_program(program);
  glrender::set_uniform_mat4(program, "projection", camera.projection);
  glrender::unuse_program();

  float prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    glrender::set_background_RGB(glrender::RGB(30, 50, 50));

    glrender::bind_texture(tex);
    glrender::use_program(program);
    glrender::bind_vao(vao);
    glrender::bind_ebo(index_buffer);

    float curr_time = glfwGetTime();
    glrender::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    glrender::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    glrender::unbind_texture();
    glrender::unbind_vao();
    glrender::unbind_ebo();
    glrender::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  glrender::delete_vao(vao);
  glrender::delete_vbo(vertex_buffer);
  glrender::delete_vbo(uv_buffer);
  glrender::delete_ebo(index_buffer);
  glrender::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}
