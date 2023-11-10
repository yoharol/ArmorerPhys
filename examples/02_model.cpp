
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include <igl/readOBJ.h>

#include "ArmorerSim/RenderCore.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  asim::init_glfw();
  GLFWwindow *window =
      asim::create_window(SCR_WIDTH, SCR_HEIGHT, "Example2: Load Model");
  asim::init_glad();

  // build and compile our shader program

  asim::Shader vertex_shader =
      asim::create_shader(asim::source::basic_shader.vertex, GL_VERTEX_SHADER);
  asim::Shader fragment_shader = asim::create_shader(
      asim::source::basic_shader.fragment, GL_FRAGMENT_SHADER);

  asim::Program program = asim::create_program(vertex_shader, fragment_shader);

  // link shaders
  asim::delete_shader(vertex_shader);
  asim::delete_shader(fragment_shader);

  asim::MatxXf V;
  asim::MatxXi F;

  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);

  int n_vertices = V.rows();
  int n_faces = F.rows();

  asim::VAO vao = asim::create_vao();
  asim::bind_vao(vao);

  asim::EBO index_buffer = asim::create_ebo();
  asim::bind_ebo(index_buffer);
  asim::set_ebo_static_data(F.data(), F.size() * sizeof(int));

  asim::VBO vertex_buffer = asim::create_vbo();
  asim::bind_vbo(vertex_buffer);
  asim::set_vbo_static_data(V.data(), V.size() * sizeof(float));

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  asim::unbind_vbo();

  asim::unbind_vao();

  // asim::set_wireframe_mode(true);
  asim::set_wireframe_mode(false);

  asim::Camera camera = asim::create_camera(
      {0.0f, 0.0f, 3.0f}, {0.0f, 0.35f, 0.0f}, {0.0f, 1.0f, 0.0f},
      float(SCR_WIDTH) / float(SCR_HEIGHT));
  // std::cout << camera.projection << std::endl;

  asim::use_program(program);
  asim::set_uniform_mat4(program, "projection", camera.projection);
  asim::unuse_program();

  float prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    asim::set_background_RGB(asim::RGB(30, 50, 50));

    asim::use_program(program);
    asim::bind_vao(vao);

    float curr_time = glfwGetTime();
    asim::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    asim::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    // glPointSize(15.0f);
    // glDrawArrays(GL_POINTS, 0, n_vertices);
    glDrawElements(GL_TRIANGLES, n_faces * 3, GL_UNSIGNED_INT, 0);

    asim::unbind_texture();
    asim::unbind_vao();
    asim::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  asim::delete_vao(vao);
  asim::delete_vbo(vertex_buffer);
  asim::delete_ebo(index_buffer);
  asim::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}
