
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include <igl/readOBJ.h>

#include "ArmorerGL.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  agl::init_glfw();
  GLFWwindow *window =
      agl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example2: Load Model");
  agl::init_glad();

  // build and compile our shader program

  agl::Shader vertex_shader = agl::create_shader(
      agl::source::basic_shader.vertex, GL_VERTEX_SHADER);
  agl::Shader fragment_shader = agl::create_shader(
      agl::source::basic_shader.fragment, GL_FRAGMENT_SHADER);

  agl::Program program =
      agl::create_program(vertex_shader, fragment_shader);

  // link shaders
  agl::delete_shader(vertex_shader);
  agl::delete_shader(fragment_shader);

  agl::MatxXf V;
  agl::MatxXi F;

  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);

  int n_vertices = V.rows();
  int n_faces = F.rows();

  agl::VAO vao = agl::create_vao();
  agl::bind_vao(vao);

  agl::EBO index_buffer = agl::create_ebo();
  agl::bind_ebo(index_buffer);
  agl::set_ebo_static_data(F.data(), F.size() * sizeof(int));

  agl::VBO vertex_buffer = agl::create_vbo();
  agl::bind_vbo(vertex_buffer);
  agl::set_vbo_static_data(V.data(), V.size() * sizeof(float));

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  agl::unbind_vbo();

  agl::unbind_vao();

  // agl::set_wireframe_mode(true);
  agl::set_wireframe_mode(false);

  agl::Camera camera = agl::create_camera(
      {0.0f, 0.0f, 3.0f}, {0.0f, 0.35f, 0.0f}, {0.0f, 1.0f, 0.0f},
      float(SCR_WIDTH) / float(SCR_HEIGHT));
  // std::cout << camera.projection << std::endl;

  agl::use_program(program);
  agl::set_uniform_mat4(program, "projection", camera.projection);
  agl::unuse_program();

  float prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    agl::set_background_RGB(agl::RGB(30, 50, 50));

    agl::use_program(program);
    agl::bind_vao(vao);

    float curr_time = glfwGetTime();
    agl::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    agl::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    // glPointSize(15.0f);
    // glDrawArrays(GL_POINTS, 0, n_vertices);
    glDrawElements(GL_TRIANGLES, n_faces * 3, GL_UNSIGNED_INT, 0);

    agl::unbind_texture();
    agl::unbind_vao();
    agl::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  agl::delete_vao(vao);
  agl::delete_vbo(vertex_buffer);
  agl::delete_ebo(index_buffer);
  agl::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}
