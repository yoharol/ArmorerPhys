#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerPhys/RenderCore.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  aphys::init_glfw();
  GLFWwindow *window = aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example1: Textured Triangles");
  aphys::init_glad();

  // build and compile our shader program

  aphys::Shader vertex_shader = aphys::create_shader(
      aphys::source::basic_uv_shader.vertex, GL_VERTEX_SHADER);
  aphys::Shader fragment_shader = aphys::create_shader(
      aphys::source::basic_uv_shader.fragment, GL_FRAGMENT_SHADER);

  aphys::Program program =
      aphys::create_program(vertex_shader, fragment_shader);

  // link shaders
  aphys::delete_shader(vertex_shader);
  aphys::delete_shader(fragment_shader);

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

  aphys::VAO vao = aphys::create_vao();
  aphys::bind_vao(vao);

  aphys::EBO index_buffer = aphys::create_ebo();
  aphys::bind_ebo(index_buffer);
  aphys::set_ebo_static_data(indices.data(), indices.size() * sizeof(int));

  aphys::VBO vertex_buffer = aphys::create_vbo();
  aphys::bind_vbo(vertex_buffer);
  aphys::set_vbo_static_data(vertices.data(), vertices.size() * sizeof(double));

  glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(double),
                        (void *)0);
  glEnableVertexAttribArray(0);
  aphys::unbind_vbo();

  aphys::VBO uv_buffer = aphys::create_vbo();
  aphys::bind_vbo(uv_buffer);
  aphys::set_vbo_static_data(texCoords.data(),
                             texCoords.size() * sizeof(double));
  glVertexAttribPointer(1, 2, GL_DOUBLE, GL_FALSE, 2 * sizeof(double),
                        (void *)0);
  glEnableVertexAttribArray(1);
  aphys::unbind_vbo();

  aphys::unbind_vao();

  aphys::Texture tex = aphys::create_texture();
  aphys::bind_texture(tex);
  aphys::set_texture_wrap(GL_CLAMP);
  aphys::set_texture_filter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
  aphys::Image img =
      aphys::load_image(std::string(ASSETS_PATH) + "/" + "awesomeface.png");
  aphys::set_texture_image(0, img, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE);
  aphys::generate_texture_mipmap();
  aphys::free_image(img);
  aphys::unbind_texture();

  // set blend mode
  aphys::set_blend_transparent();
  aphys::set_wireframe_mode(false);

  aphys::Camera camera = aphys::create_camera(
      {0.0f, 0.0f, 3.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
      double(SCR_WIDTH) / double(SCR_HEIGHT));

  aphys::use_program(program);
  aphys::set_uniform_mat4(program, "projection", camera.projection);
  aphys::unuse_program();

  double prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    aphys::set_background_RGB(aphys::RGB(30, 50, 50));

    aphys::bind_texture(tex);
    aphys::use_program(program);
    aphys::bind_vao(vao);
    aphys::bind_ebo(index_buffer);

    double curr_time = glfwGetTime();
    aphys::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    aphys::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    aphys::unbind_texture();
    aphys::unbind_vao();
    aphys::unbind_ebo();
    aphys::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  aphys::delete_vao(vao);
  aphys::delete_vbo(vertex_buffer);
  aphys::delete_vbo(uv_buffer);
  aphys::delete_ebo(index_buffer);
  aphys::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}
