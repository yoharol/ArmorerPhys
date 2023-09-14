#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  armgl::init_glfw();
  GLFWwindow *window = armgl::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example1: Textured Triangles");
  armgl::init_glad();

  // build and compile our shader program

  armgl::Shader vertex_shader = armgl::create_shader(
      armgl::source::basic_uv_shader.vertex, GL_VERTEX_SHADER);
  armgl::Shader fragment_shader = armgl::create_shader(
      armgl::source::basic_uv_shader.fragment, GL_FRAGMENT_SHADER);

  armgl::Program program =
      armgl::create_program(vertex_shader, fragment_shader);

  // link shaders
  armgl::delete_shader(vertex_shader);
  armgl::delete_shader(fragment_shader);

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

  armgl::VAO vao = armgl::create_vao();
  armgl::bind_vao(vao);

  armgl::EBO index_buffer = armgl::create_ebo();
  armgl::bind_ebo(index_buffer);
  armgl::set_ebo_static_data(indices.data(), indices.size() * sizeof(int));

  armgl::VBO vertex_buffer = armgl::create_vbo();
  armgl::bind_vbo(vertex_buffer);
  armgl::set_vbo_static_data(vertices.data(), vertices.size() * sizeof(float));

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  armgl::unbind_vbo();

  armgl::VBO uv_buffer = armgl::create_vbo();
  armgl::bind_vbo(uv_buffer);
  armgl::set_vbo_static_data(texCoords.data(),
                             texCoords.size() * sizeof(float));
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  armgl::unbind_vbo();

  armgl::unbind_vao();

  armgl::Texture tex = armgl::create_texture();
  armgl::bind_texture(tex);
  armgl::set_texture_wrap(GL_CLAMP);
  armgl::set_texture_filter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
  armgl::Image img =
      armgl::load_image(std::string(ASSETS_PATH) + "/" + "awesomeface.png");
  armgl::set_texture_image(0, img, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE);
  armgl::generate_texture_mipmap();
  armgl::free_image(img);
  armgl::unbind_texture();

  // set blend mode
  armgl::set_blend_transparent();
  armgl::set_wireframe_mode(false);

  armgl::Camera camera = armgl::create_camera(
      {0.0f, 0.0f, 3.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
      float(SCR_WIDTH) / float(SCR_HEIGHT));

  armgl::use_program(program);
  armgl::set_uniform_mat4(program, "projection", camera.projection);
  armgl::unuse_program();

  float prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    armgl::set_background_RGB(armgl::RGB(30, 50, 50));

    armgl::bind_texture(tex);
    armgl::use_program(program);
    armgl::bind_vao(vao);
    armgl::bind_ebo(index_buffer);

    float curr_time = glfwGetTime();
    armgl::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    armgl::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    armgl::unbind_texture();
    armgl::unbind_vao();
    armgl::unbind_ebo();
    armgl::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  armgl::delete_vao(vao);
  armgl::delete_vbo(vertex_buffer);
  armgl::delete_vbo(uv_buffer);
  armgl::delete_ebo(index_buffer);
  armgl::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}
