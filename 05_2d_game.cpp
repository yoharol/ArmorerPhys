#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "gl_render.h"

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;

int main() {
  GLFWwindow* window =
      glrender::create_window(SCR_WIDTH, SCR_HEIGHT, "Example5: 2d game");
  float bottom = 0.0f;
  float top = 2.0f;
  float left = 0.0f;
  float right = 2.0f;

  glrender::Scene scene = glrender::create_scene(
      glrender::Light{
          {242, 242, 242},     // light color
          {9, 5, 88},          // ambient color
          {0.0f, 0.35f, 5.0f}  // light position
      },
      glrender::create_camera(
          {0.0f, 0.0f, -2.0f},                  // camera position
          {0.0f, 0.35f, 0.0f},                  // camera target
          {0.0f, 1.0f, 0.0f},                   // camera up axis
          float(SCR_WIDTH) / float(SCR_HEIGHT)  // camera aspect
          ));
  glrender::set_2d_camera(scene.camera, left, right, bottom, top);
  glrender::Gui gui = glrender::create_gui(window, "gui");

  glrender::Points points{{}, {}, {255, 0, 0}, 25.0f};
  glrender::ContinuousLines lines{{}, {}, {0, 205, 205}, 10.0f, true};

  glrender::Mat2f v_p;
  v_p.resize(2, 2);
  v_p << 1.0f, 1.0f,  //
      1.0f, 0.5f;     //
  glrender::Mat2f v_p_ref = v_p;
  float l0 = (v_p.row(0) - v_p.row(1)).norm();
  glrender::Mat2f v_vel;
  v_vel.resize(2, 2);
  v_vel << 0.0f, 0.0f,  //
      0.0f, 0.0f;       //
  glrender::set_points_data(points, v_p, {});
  glrender::set_continuous_lines_data(lines, v_p, {});

  glrender::Vec2f gravity(0.0f, -10.0f);
  float mass = 1.0f;
  float stiffness = 50.0f;
  float damping = 0.5f;

  auto reset = [&]() {
    v_p = v_p_ref;
    v_vel.setZero();
    glrender::set_points_data(points, v_p, {});
    glrender::set_continuous_lines_data(lines, v_p, {});
  };

  glrender::add_gui_func(gui,
                         [&gravity, &stiffness, &mass, &damping, &reset]() {
                           ImGui::Text("Parameters:");
                           ImGui::InputFloat2("g", gravity.data());
                           ImGui::InputFloat("k", &stiffness);
                           ImGui::InputFloat("m", &mass);
                           ImGui::InputFloat("beta", &damping);
                           if (ImGui::Button("Reset")) {
                             reset();
                           }
                         });
  glrender::add_render_func(scene, glrender::get_render_func(points));
  glrender::add_render_func(scene, glrender::get_render_func(lines));

  glfwSwapInterval(1);

  float prev_time = glfwGetTime();
  float start_time = prev_time;

  while (!glfwWindowShouldClose(window)) {
    glrender::set_background_RGB({244, 244, 244});

    float delta_time = glfwGetTime() - prev_time;
    prev_time = glfwGetTime();
    float dt = delta_time / 10.0f;

    for (int _ = 0; _ < 10; _++) {
      glrender::Vec2f dx = v_p.row(0) - v_p.row(1);
      glrender::Vec2f force =
          gravity * mass + stiffness * (dx.norm() - l0) * dx;
      glrender::Vec2f acc = force / mass;
      v_vel.row(1).transpose() += acc * dt;
      v_vel *= exp(-damping * dt);
      v_p += v_vel * dt;
    }

    glrender::set_points_data(points, v_p, {});
    glrender::set_continuous_lines_data(lines, v_p, {});

    // glrender::Mat2fToList3f(v_p, pos);
    glrender::render_scene(scene);
    glrender::render_gui(gui);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  glrender::destroy_gui(gui);
  glfwTerminate();
}