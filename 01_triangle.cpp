#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  agl::init_glfw();
  GLFWwindow *window = agl::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example1: Textured Triangles");
  agl::init_glad();

  // build and compile our shader program

  agl::Shader vertex_shader = agl::create_shader(
      agl::source::basic_uv_shader.vertex, GL_VERTEX_SHADER);
  agl::Shader fragment_shader = agl::create_shader(
      agl::source::basic_uv_shader.fragment, GL_FRAGMENT_SHADER);

  agl::Program program =
      agl::create_program(vertex_shader, fragment_shader);

  // link shaders
  agl::delete_shader(vertex_shader);
  agl::delete_shader(fragment_shader);

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

  agl::VAO vao = agl::create_vao();
  agl::bind_vao(vao);

  agl::EBO index_buffer = agl::create_ebo();
  agl::bind_ebo(index_buffer);
  agl::set_ebo_static_data(indices.data(), indices.size() * sizeof(int));

  agl::VBO vertex_buffer = agl::create_vbo();
  agl::bind_vbo(vertex_buffer);
  agl::set_vbo_static_data(vertices.data(), vertices.size() * sizeof(float));

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  agl::unbind_vbo();

  agl::VBO uv_buffer = agl::create_vbo();
  agl::bind_vbo(uv_buffer);
  agl::set_vbo_static_data(texCoords.data(),
                             texCoords.size() * sizeof(float));
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  agl::unbind_vbo();

  agl::unbind_vao();

  agl::Texture tex = agl::create_texture();
  agl::bind_texture(tex);
  agl::set_texture_wrap(GL_CLAMP);
  agl::set_texture_filter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
  agl::Image img =
      agl::load_image(std::string(ASSETS_PATH) + "/" + "awesomeface.png");
  agl::set_texture_image(0, img, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE);
  agl::generate_texture_mipmap();
  agl::free_image(img);
  agl::unbind_texture();

  // set blend mode
  agl::set_blend_transparent();
  agl::set_wireframe_mode(false);

  agl::Camera camera = agl::create_camera(
      {0.0f, 0.0f, 3.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
      float(SCR_WIDTH) / float(SCR_HEIGHT));

  agl::use_program(program);
  agl::set_uniform_mat4(program, "projection", camera.projection);
  agl::unuse_program();

  float prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    agl::set_background_RGB(agl::RGB(30, 50, 50));

    agl::bind_texture(tex);
    agl::use_program(program);
    agl::bind_vao(vao);
    agl::bind_ebo(index_buffer);

    float curr_time = glfwGetTime();
    agl::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    agl::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    agl::unbind_texture();
    agl::unbind_vao();
    agl::unbind_ebo();
    agl::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  agl::delete_vao(vao);
  agl::delete_vbo(vertex_buffer);
  agl::delete_vbo(uv_buffer);
  agl::delete_ebo(index_buffer);
  agl::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}
