#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <iostream>

#include "ArmorerPhys/RenderCore.h"
#include "ArmorerPhys/SimCore.h"

const unsigned int SCR_WIDTH = 700;
const unsigned int SCR_HEIGHT = 700;

int main() {
  GLFWwindow* window =
      aphys::create_window(SCR_WIDTH, SCR_HEIGHT, "Example6: handle input");
  float bottom = 0.0f;
  float top = 1.0f;
  float left = 0.0f;
  float right = 1.0f;

  aphys::Scene scene =
      aphys::create_scene(aphys::default_light, aphys::default_camera);
  aphys::set_2d_camera(scene.camera, left, right, bottom, top);

  aphys::Matx2f v_p;
  aphys::Matx3i face_indices;
  aphys::create_rectangle(0.4, 0.6, 2, 0.4, 0.6, 3, v_p, face_indices);
  aphys::Matx2i edge_indices;
  aphys::Vecxf rest_length;
  aphys::extract_edge(face_indices, edge_indices);

  aphys::Points points = aphys::create_points();
  aphys::set_points_data(points, v_p, aphys::MatxXf());
  points.color = aphys::RGB(255, 0, 0);
  points.point_size = 4.0f;
  aphys::Edges edges = aphys::create_edges();
  aphys::set_edges_data(edges, v_p, edge_indices, aphys::MatxXf());
  edges.color = aphys::RGB(0, 0, 0);
  edges.width = 1.0f;

  aphys::add_render_func(scene, aphys::get_render_func(points));
  aphys::add_render_func(scene, aphys::get_render_func(edges));

  aphys::InputHandler& handler = aphys::create_input_handler(window);
  aphys::add_mouse_move_func(handler, [&](aphys::InputHandler& input_handler) {
    if (input_handler.left_pressing) {
      float xpos, ypos;
      xpos = left + (right - left) * handler.xpos;
      ypos = bottom + (top - bottom) * handler.ypos;
      v_p.row(0) = aphys::Vec2f(xpos, ypos);
    }
  });
  aphys::add_mouse_input_func(handler, [](aphys::InputHandler& input_handler,
                                          int button, int action) {
    std::cout << "mouse button event: " << button << " " << action << std::endl;
  });
  aphys::add_key_input_func(
      handler, [](aphys::InputHandler& input_handler, int key, int action) {
        std::cout << "key event: " << key << " " << action << std::endl;
      });

  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    aphys::set_background_RGB({244, 244, 244});

    aphys::set_points_data(points, v_p, aphys::MatxXf());
    aphys::set_edges_data(edges, v_p, edge_indices, aphys::MatxXf());

    aphys::render_scene(scene);

    glfwSwapBuffers(window);
  }
  glfwTerminate();
}