#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window = armgl::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example7: Affine2d Visualization");

  armgl::Scene scene =
      armgl::create_scene(armgl::default_light, armgl::default_camera);
  armgl::set_2d_camera(scene.camera, -1.0f, 2.0f, -1.0f, 2.0f);
  armgl::Gui gui = armgl::create_gui(window, "gui");
  gui.width = 200;
  gui.height = 200;

  armgl::Points points = armgl::create_points();
  armgl::Lines lines = armgl::create_lines();

  armgl::Matx2f v_p;
  armgl::Matx2f v_p_ref;
  v_p.resize(4, 2);
  v_p << 0.0f, 0.0f,  //
      1.0f, 0.0f,     //
      1.0f, 1.0f,     //
      0.0f, 1.0f;     //
  v_p_ref = v_p;
  armgl::Mat2f A = armgl::Mat2f::Identity();

  armgl::set_points_data(points, v_p, armgl::MatxXf());
  points.color = armgl::RGB(255, 0, 0);
  points.point_size = 30.0f;
  armgl::set_lines_data(lines, v_p, armgl::MatxXf());
  lines.mode = GL_LINE_LOOP;
  lines.color = armgl::RGB(0, 0, 255);

  armgl::add_render_func(scene, armgl::get_render_func(points));
  armgl::add_render_func(scene, armgl::get_render_func(lines));

  armgl::add_gui_func(gui, [&A]() {
    ImGui::Text("Affine Matrix:");
    ImGui::SetNextItemWidth(50);
    ImGui::DragFloat("a11", &A(0, 0), 0.0f, -1.0f, 1.0f);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50);
    ImGui::DragFloat("a12", &A(0, 1), 0.0f, -1.0f, 1.0f);

    ImGui::SetNextItemWidth(50);
    ImGui::DragFloat("a21", &A(1, 0), 0.0f, -1.0f, 1.0f);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50);
    ImGui::DragFloat("a22", &A(1, 1), 0.0f, -1.0f, 1.0f);
  });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    armgl::handle_gui_input(gui);

    armgl::set_background_RGB({244, 244, 244});

    armgl::set_points_data(points, v_p_ref * A.transpose(), armgl::MatxXf());

    armgl::render_scene(scene);
    armgl::render_gui(gui);

    glfwSwapBuffers(window);
  }
  armgl::destroy_gui(gui);
  glfwTerminate();
}