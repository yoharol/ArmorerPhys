
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include <igl/readMESH.h>

#include "gl_render.h"

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  glrender::init_glfw();
  GLFWwindow *window =
      glrender::create_window(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL");
  glrender::init_glad();

  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

  // build and compile our shader program

  glrender::Shader vertex_shader = glrender::create_shader(
      glrender::source::basic_shader.vertex, GL_VERTEX_SHADER);
  glrender::Shader fragment_shader = glrender::create_shader(
      glrender::source::basic_shader.fragment, GL_FRAGMENT_SHADER);

  glrender::Program program =
      glrender::create_program(vertex_shader, fragment_shader);

  // link shaders
  glrender::delete_shader(vertex_shader);
  glrender::delete_shader(fragment_shader);

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> V;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> F, T;

  igl::readMESH(std::string(ASSETS_PATH) + "/spot.mesh", V, T, F);
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> F2(
      F.rows() / 2, F.cols());
  for (int i = 0; i < F.rows() / 2; i++) {
    for (int j = 0; j < F.cols(); j++) {
      F2(i, j) = F(i * 2, j);
    }
  }

  int n_vertices = V.rows();
  int n_faces = F2.rows();

  glrender::VAO vao = glrender::create_vao();
  glrender::bind_vao(vao);

  glrender::EBO index_buffer = glrender::create_ebo();
  glrender::bind_ebo(index_buffer);
  glrender::set_ebo_static_data(F2.data(), F2.size() * sizeof(int));

  glrender::VBO vertex_buffer = glrender::create_vbo();
  glrender::bind_vbo(vertex_buffer);
  glrender::set_vbo_static_data(V.data(), V.size() * sizeof(float));

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glrender::unbind_vbo();

  glrender::unbind_vao();

  // glrender::set_wireframe_mode(true);
  glrender::set_wireframe_mode(false);

  glrender::Camera camera = glrender::create_camera();
  camera.position = Eigen::Vector3f(0.0f, 0.0f, 3.0f);
  camera.lookat = Eigen::Vector3f(0.0f, 0.35f, 0.0f);
  camera.up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  camera.aspect = float(SCR_WIDTH) / float(SCR_HEIGHT);
  glrender::update_camera(camera);
  // std::cout << camera.projection << std::endl;

  glrender::use_program(program);
  glrender::set_uniform_mat4(program, "projection", camera.projection);
  glrender::unuse_program();

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CW);

  float prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    processInput(window);

    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glrender::use_program(program);
    glrender::bind_vao(vao);

    float curr_time = glfwGetTime();
    glrender::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    glrender::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    // glPointSize(15.0f);
    // glDrawArrays(GL_POINTS, 0, n_vertices);
    glDrawElements(GL_TRIANGLES, n_faces * 3, GL_UNSIGNED_INT, 0);

    glrender::unbind_texture();
    glrender::unbind_vao();
    glrender::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  glrender::delete_vao(vao);
  glrender::delete_vbo(vertex_buffer);
  glrender::delete_ebo(index_buffer);
  glrender::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}

void processInput(GLFWwindow *window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
}