#include "ArmorerPhys/render/scene.h"

#include <functional>

#include "ArmorerPhys/type.h"
#include "ArmorerPhys/render/buffer.h"
#include "ArmorerPhys/render/shader.h"
#include "ArmorerPhys/render/camera.h"
#include "ArmorerPhys/geom.h"

namespace aphys {

Scene create_scene(Light light, Camera camera) {
  Scene scene;
  scene.light = light;
  scene.camera = camera;
  scene.render_funcs = std::vector<RenderFunc>();
  scene.depth_test = std::vector<bool>();
  scene.delta_time = 0.0;
  scene.time = glfwGetTime();
  return scene;
}

void add_render_func(Scene& scene, RenderFunc func, bool depth_test) {
  scene.render_funcs.push_back(func);
  scene.depth_test.push_back(depth_test);
}

void render_scene(Scene& scene) {
  for (int i = 0; i < scene.render_funcs.size(); ++i) {
    if (scene.depth_test[i]) {
      glEnable(GL_DEPTH_TEST);
    } else {
      glDisable(GL_DEPTH_TEST);
    }
    scene.render_funcs[i](scene);
  }
  scene.delta_time = glfwGetTime() - scene.time;
  scene.time = glfwGetTime();
}

InputHandler& create_input_handler(GLFWwindow* window) {
  InputHandler& handler = InputHandler::getInstance();
  glfwSetCursorPosCallback(
      window, [](GLFWwindow* window, double xpos, double ypos) {
        InputHandler& handler = InputHandler::getInstance();
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        handler.xpos = static_cast<float>(xpos / width);
        handler.ypos = static_cast<float>(1.0 - ypos / height);
        for (auto func : handler.mouse_move_funcs) {
          func(handler);
        }
      });
  glfwSetMouseButtonCallback(
      window, [](GLFWwindow* window, int button, int action, int mods) {
        InputHandler& handler = InputHandler::getInstance();
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
          handler.left_pressing = true;
        }
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
          handler.left_pressing = false;
        }
        for (auto func : handler.mouse_input_funcs) {
          func(handler, button, action);
        }
      });
  glfwSetKeyCallback(window, [](GLFWwindow* window, int key, int scancode,
                                int action, int mods) {
    InputHandler& handler = InputHandler::getInstance();
    for (auto func : handler.key_input_funcs) {
      func(handler, key, action);
    }
  });
  return handler;
}

void add_mouse_move_func(InputHandler& handler, MouseMoveFunc func) {
  handler.mouse_move_funcs.push_back(func);
}

void add_mouse_input_func(InputHandler& handler, MouseInputFunc func) {
  handler.mouse_input_funcs.push_back(func);
}

void add_key_input_func(InputHandler& handler, KeyInputFunc func) {
  handler.key_input_funcs.push_back(func);
}

}  // namespace aphys
