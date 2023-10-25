#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerGL.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window = agl::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example7: Affine2d Visualization");

  agl::Scene scene =
      agl::create_scene(agl::default_light, agl::default_camera);
  agl::set_2d_camera(scene.camera, -1.0f, 2.0f, -1.0f, 2.0f);
  agl::Gui gui = agl::create_gui(window, "gui");
  gui.width = 200;
  gui.height = 200;

  agl::Points points = agl::create_points();
  agl::Lines lines = agl::create_lines();

  agl::Matx2f v_p;
  agl::Matx2f v_p_ref;
  v_p.resize(4, 2);
  v_p << 0.0f, 0.0f,  //
      1.0f, 0.0f,     //
      1.0f, 1.0f,     //
      0.0f, 1.0f;     //
  v_p_ref = v_p;
  agl::Mat2f A = agl::Mat2f::Identity();

  agl::set_points_data(points, v_p, agl::MatxXf());
  points.color = agl::RGB(255, 0, 0);
  points.point_size = 30.0f;
  agl::set_lines_data(lines, v_p, agl::MatxXf());
  lines.mode = GL_LINE_LOOP;
  lines.color = agl::RGB(0, 0, 255);

  agl::add_render_func(scene, agl::get_render_func(points));
  agl::add_render_func(scene, agl::get_render_func(lines));

  agl::add_gui_func(gui, [&A]() {
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
    agl::handle_gui_input(gui);

    agl::set_background_RGB({244, 244, 244});

    agl::set_points_data(points, v_p_ref * A.transpose(), agl::MatxXf());

    agl::render_scene(scene);
    agl::render_gui(gui);

    glfwSwapBuffers(window);
  }
  agl::destroy_gui(gui);
  glfwTerminate();
}