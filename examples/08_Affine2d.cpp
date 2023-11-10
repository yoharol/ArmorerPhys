#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerSim/RenderCore.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window = asim::create_window(SCR_WIDTH, SCR_HEIGHT,
                                           "Example8: Affine2d Visualization");

  asim::Scene scene =
      asim::create_scene(asim::default_light, asim::default_camera);
  asim::set_2d_camera(scene.camera, -1.0f, 2.0f, -1.0f, 2.0f);
  asim::Gui gui = asim::create_gui(window, "gui");
  gui.width = 200;
  gui.height = 200;

  asim::Points points = asim::create_points();
  asim::Lines lines = asim::create_lines();

  asim::Matx2f v_p;
  asim::Matx2f v_p_ref;
  v_p.resize(4, 2);
  v_p << 0.0f, 0.0f,  //
      1.0f, 0.0f,     //
      1.0f, 1.0f,     //
      0.0f, 1.0f;     //
  v_p_ref = v_p;
  asim::Mat2f A = asim::Mat2f::Identity();

  asim::set_points_data(points, v_p, asim::MatxXf());
  points.color = asim::RGB(255, 0, 0);
  points.point_size = 30.0f;
  asim::set_lines_data(lines, v_p, asim::MatxXf());
  lines.mode = GL_LINE_LOOP;
  lines.color = asim::RGB(0, 0, 255);

  asim::add_render_func(scene, asim::get_render_func(points));
  asim::add_render_func(scene, asim::get_render_func(lines));

  asim::add_gui_func(gui, [&A]() {
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
    asim::handle_gui_input(gui);

    asim::set_background_RGB({244, 244, 244});

    asim::set_points_data(points, v_p_ref * A.transpose(), asim::MatxXf());

    asim::render_scene(scene);
    asim::render_gui(gui);

    glfwSwapBuffers(window);
  }
  asim::destroy_gui(gui);
  glfwTerminate();
}