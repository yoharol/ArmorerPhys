#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerPhys/RenderCore.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window = aphys::create_window(SCR_WIDTH, SCR_HEIGHT,
                                            "Example8: Affine2d Visualization");

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, -1.0f, 2.0f, -1.0f, 2.0f);
  aphys::Gui gui = aphys::create_gui(window, "gui");
  gui.width = 200;
  gui.height = 200;

  aphys::Points points = aphys::create_points();
  aphys::Lines lines = aphys::create_lines();

  aphys::Matx2f v_p;
  aphys::Matx2f v_p_ref;
  v_p.resize(4, 2);
  v_p << 0.0f, 0.0f,  //
      1.0f, 0.0f,     //
      1.0f, 1.0f,     //
      0.0f, 1.0f;     //
  v_p_ref = v_p;
  aphys::Mat2f A = aphys::Mat2f::Identity();

  aphys::set_points_data(points, v_p, aphys::MatxXf());
  points.color = aphys::RGB(255, 0, 0);
  points.point_size = 30.0f;
  aphys::set_lines_data(lines, v_p, aphys::MatxXf());
  lines.mode = GL_LINE_LOOP;
  lines.color = aphys::RGB(0, 0, 255);

  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(lines));

  aphys::add_gui_func(gui, [&A]() {
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
    aphys::handle_gui_input(gui);

    aphys::set_background_RGB({244, 244, 244});

    aphys::set_points_data(points, v_p_ref * A.transpose(), aphys::MatxXf());

    aphys::render_scene(scene);
    aphys::render_gui(gui);

    glfwSwapBuffers(window);
  }
  aphys::destroy_gui(gui);
  glfwTerminate();
}