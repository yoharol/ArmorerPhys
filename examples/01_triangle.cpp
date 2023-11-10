#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerSim/RenderCore.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  asim::init_glfw();
  GLFWwindow *window = asim::create_window(SCR_WIDTH, SCR_HEIGHT,
                                           "Example1: Textured Triangles");
  asim::init_glad();

  // build and compile our shader program

  asim::Shader vertex_shader = asim::create_shader(
      asim::source::basic_uv_shader.vertex, GL_VERTEX_SHADER);
  asim::Shader fragment_shader = asim::create_shader(
      asim::source::basic_uv_shader.fragment, GL_FRAGMENT_SHADER);

  asim::Program program = asim::create_program(vertex_shader, fragment_shader);

  // link shaders
  asim::delete_shader(vertex_shader);
  asim::delete_shader(fragment_shader);

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

  asim::VAO vao = asim::create_vao();
  asim::bind_vao(vao);

  asim::EBO index_buffer = asim::create_ebo();
  asim::bind_ebo(index_buffer);
  asim::set_ebo_static_data(indices.data(), indices.size() * sizeof(int));

  asim::VBO vertex_buffer = asim::create_vbo();
  asim::bind_vbo(vertex_buffer);
  asim::set_vbo_static_data(vertices.data(), vertices.size() * sizeof(float));

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  asim::unbind_vbo();

  asim::VBO uv_buffer = asim::create_vbo();
  asim::bind_vbo(uv_buffer);
  asim::set_vbo_static_data(texCoords.data(), texCoords.size() * sizeof(float));
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  asim::unbind_vbo();

  asim::unbind_vao();

  asim::Texture tex = asim::create_texture();
  asim::bind_texture(tex);
  asim::set_texture_wrap(GL_CLAMP);
  asim::set_texture_filter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
  asim::Image img =
      asim::load_image(std::string(ASSETS_PATH) + "/" + "awesomeface.png");
  asim::set_texture_image(0, img, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE);
  asim::generate_texture_mipmap();
  asim::free_image(img);
  asim::unbind_texture();

  // set blend mode
  asim::set_blend_transparent();
  asim::set_wireframe_mode(false);

  asim::Camera camera = asim::create_camera(
      {0.0f, 0.0f, 3.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
      float(SCR_WIDTH) / float(SCR_HEIGHT));

  asim::use_program(program);
  asim::set_uniform_mat4(program, "projection", camera.projection);
  asim::unuse_program();

  float prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    asim::set_background_RGB(asim::RGB(30, 50, 50));

    asim::bind_texture(tex);
    asim::use_program(program);
    asim::bind_vao(vao);
    asim::bind_ebo(index_buffer);

    float curr_time = glfwGetTime();
    asim::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    asim::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    asim::unbind_texture();
    asim::unbind_vao();
    asim::unbind_ebo();
    asim::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  asim::delete_vao(vao);
  asim::delete_vbo(vertex_buffer);
  asim::delete_vbo(uv_buffer);
  asim::delete_ebo(index_buffer);
  asim::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}
