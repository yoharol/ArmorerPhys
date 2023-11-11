
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include <igl/readOBJ.h>

#include "ArmorerPhys/RenderCore.h"

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  aphys::init_glfw();
  GLFWwindow *window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT, "Example2: Load Model");
  aphys::init_glad();

  // build and compile our shader program

  aphys::Shader vertex_shader = aphys::create_shader(
      aphys::source::basic_shader.vertex, GL_VERTEX_SHADER);
  aphys::Shader fragment_shader = aphys::create_shader(
      aphys::source::basic_shader.fragment, GL_FRAGMENT_SHADER);

  aphys::Program program =
      aphys::create_program(vertex_shader, fragment_shader);

  // link shaders
  aphys::delete_shader(vertex_shader);
  aphys::delete_shader(fragment_shader);

  aphys::MatxXf V;
  aphys::MatxXi F;

  igl::readOBJ(std::string(ASSETS_PATH) + "/spot.obj", V, F);

  int n_vertices = V.rows();
  int n_faces = F.rows();

  aphys::VAO vao = aphys::create_vao();
  aphys::bind_vao(vao);

  aphys::EBO index_buffer = aphys::create_ebo();
  aphys::bind_ebo(index_buffer);
  aphys::set_ebo_static_data(F.data(), F.size() * sizeof(int));

  aphys::VBO vertex_buffer = aphys::create_vbo();
  aphys::bind_vbo(vertex_buffer);
  aphys::set_vbo_static_data(V.data(), V.size() * sizeof(float));

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  aphys::unbind_vbo();

  aphys::unbind_vao();

  // aphys::set_wireframe_mode(true);
  aphys::set_wireframe_mode(false);

  aphys::Camera camera = aphys::create_camera(
      {0.0f, 0.0f, 3.0f}, {0.0f, 0.35f, 0.0f}, {0.0f, 1.0f, 0.0f},
      float(SCR_WIDTH) / float(SCR_HEIGHT));
  // std::cout << camera.projection << std::endl;

  aphys::use_program(program);
  aphys::set_uniform_mat4(program, "projection", camera.projection);
  aphys::unuse_program();

  float prev_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
    aphys::set_background_RGB(aphys::RGB(30, 50, 50));

    aphys::use_program(program);
    aphys::bind_vao(vao);

    float curr_time = glfwGetTime();
    aphys::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    aphys::set_uniform_mat4(program, "projection", camera.projection);

    // glDrawArrays(GL_TRIANGLES, 0, 3);
    // glPointSize(15.0f);
    // glDrawArrays(GL_POINTS, 0, n_vertices);
    glDrawElements(GL_TRIANGLES, n_faces * 3, GL_UNSIGNED_INT, 0);

    aphys::unbind_texture();
    aphys::unbind_vao();
    aphys::unuse_program();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // optional: de-allocate all resources once they've outlived their purpose:
  // ------------------------------------------------------------------------
  aphys::delete_vao(vao);
  aphys::delete_vbo(vertex_buffer);
  aphys::delete_ebo(index_buffer);
  aphys::delete_program(program);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
  return 0;
}
