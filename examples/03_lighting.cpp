#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>
#include <functional>

#include <igl/readOBJ.h>

#include "ArmorerPhys/RenderCore.h"

typedef std::function<void(int)> RenderFunc;

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
  aphys::init_glfw();
  GLFWwindow *window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT, "Example3: Lighting");
  aphys::init_glad();

  aphys::Shader vertex_shader = aphys::create_shader(
      aphys::source::basic_diffuse_shader.vertex, GL_VERTEX_SHADER);
  aphys::Shader fragment_shader = aphys::create_shader(
      aphys::source::basic_diffuse_shader.fragment, GL_FRAGMENT_SHADER);

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

  aphys::VBO normal_buffer = aphys::create_vbo();
  aphys::bind_vbo(normal_buffer);
  aphys::MatxXf normals = aphys::get_normals(V, F);
  aphys::set_vbo_static_data(normals.data(), normals.size() * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(1);
  aphys::unbind_vbo();

  aphys::unbind_vao();

  // aphys::set_wireframe_mode(true);
  aphys::set_wireframe_mode(false);

  aphys::Camera camera = aphys::create_camera(
      {2.0f, 0.0f, -2.0f}, {0.0f, 0.35f, 0.0f}, {0.0f, 1.0f, 0.0f},
      float(SCR_WIDTH) / float(SCR_HEIGHT));

  // std::cout << camera.projection << std::endl;

  aphys::use_program(program);
  aphys::set_uniform_mat4(program, "projection", camera.projection);
  // aphys::set_uniform_mat4(program, "view", camera.view);
  aphys::set_uniform_RGB(program, "lightColor", aphys::RGB(242, 76, 61));
  aphys::set_uniform_RGB(program, "diffuseColor",
                            aphys::RGB(244, 244, 244));
  aphys::set_uniform_RGB(program, "specularColor",
                            aphys::RGB(255, 255, 255));
  aphys::set_uniform_RGB(program, "ambientColor", aphys::RGB(9, 5, 128));
  aphys::set_uniform_float(program, "specularStrength", 0.5f);
  aphys::set_uniform_float3(program, "lightPos",
                               aphys::Vec3f(0.0f, 0.35f, 5.0f));
  aphys::unuse_program();

  RenderFunc render_mesh = [&](int idx) {
    aphys::bind_vao(vao);
    glDrawElements(GL_TRIANGLES, n_faces * 3, GL_UNSIGNED_INT, 0);
    aphys::unbind_vao();
  };

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    aphys::set_background_RGB(aphys::RGB(250, 240, 228));

    aphys::use_program(program);
    // aphys::bind_vao(vao);

    float curr_time = glfwGetTime();
    aphys::orbit_camera_control(window, camera, 10.0, curr_time - prev_time);
    prev_time = curr_time;
    aphys::set_uniform_mat4(program, "projection", camera.projection);
    aphys::set_uniform_float3(program, "viewPos", camera.position);
    aphys::set_uniform_float3(
        program, "lightPos",
        aphys::Vec3f(5.0f * sin((curr_time - start_time) * 1.0f), 0.35f,
                        5.0f * cos((curr_time - start_time) * 1.0f)));

    render_mesh(0);

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
