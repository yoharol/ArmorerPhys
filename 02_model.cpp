
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
  armgl::init_glfw();
  GLFWwindow *window =
      armgl::create_window(SCR_WIDTH, SCR_HEIGHT, "Example2: Load Model");
  armgl::init_glad();

  // build and compile our shader program

  armgl::Shader vertex_shader = armgl::create_shader(
      armgl::source::basic_shader.vertex, GL_VERTEX_SHADER);
  armgl::Shader fragment_shader = armgl::create_shader(
      armgl::source::basic_shader.fragment, GL_FRAGMENT_SHADER);

  armgl::Program program =
      armgl::create_program(vertex_shader, fragment_shader);

  // link shaders
  armgl::delete_shader(vertex_shader);
  armgl::delete_shader(fragment_shader);

  armgl::MatXf V;
  armgl::MatXi F;

  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);

  int n_vertices = V.rows();
  int n_faces = F.rows();

  armgl::VAO vao = armgl::create_vao();
  armgl::bind_vao(vao);

  armgl::EBO index_buffer = armgl::create_ebo();
  armgl::bind_ebo(index_buffer);
  armgl::set_ebo_static_data(F.data(), F.size() * sizeof(int));

  armgl::VBO vertex_buffer = armgl::create_vbo();
  armgl::bind_vbo(vertex_buffer);
  armgl::set_vbo_static_data(V.data(), V.size() * sizeof(float));

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  armgl::unbind_vbo();

  armgl::unbind_vao();

  // armgl::set_wireframe_mode(true);
  armgl::set_wireframe_mode(false);

  armgl::Camera camera = armgl::create_camera(
      {0.0f, 0.0f, 3.0f}, {0.0f, 0.35f, 0.0f}, {0.0f, 1.0f, 0.0f},
      float(SCR_WIDTH) / float(SCR_HEIGHT));
  // std::cout << camera.projection << std::endl;

  armgl::use_program(program);
  armgl::set_uniform_mat4(program, "projection", camera.projection);
  armgl::unuse_program();

  float prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    armgl::set_background_RGB(armgl::RGB(30, 50, 50));

    armgl::use_program(program);
    armgl::bind_vao(vao);

    float curr_time = glfwGetTime();
    armgl::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    armgl::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    // glPointSize(15.0f);
    // glDrawArrays(GL_POINTS, 0, n_vertices);
    glDrawElements(GL_TRIANGLES, n_faces * 3, GL_UNSIGNED_INT, 0);

    armgl::unbind_texture();
    armgl::unbind_vao();
    armgl::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  armgl::delete_vao(vao);
  armgl::delete_vbo(vertex_buffer);
  armgl::delete_ebo(index_buffer);
  armgl::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}
