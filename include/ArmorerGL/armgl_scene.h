#ifndef GL_WARP_H_
#define GL_WARP_H_

#include <functional>

#include "armgl_type.h"
#include "armgl_buffer.h"
#include "armgl_shader.h"
#include "armgl_camera.h"
#include "armgl_geom.h"

namespace armgl {

struct Scene;
struct InputHandler;
typedef std::function<void(Scene)> RenderFunc;
typedef std::function<void(InputHandler&, int, int)> MouseInputFunc;
typedef std::function<void(InputHandler&)> MouseMoveFunc;
typedef std::function<void(InputHandler&, int, int)> KeyInputFunc;

struct Scene {
  Light light;
  Camera camera;
  std::vector<RenderFunc> render_funcs;
  std::vector<bool> depth_test;
};

Scene create_scene(Light light, Camera camera) {
  Scene scene;
  scene.light = light;
  scene.camera = camera;
  scene.render_funcs = std::vector<RenderFunc>();
  scene.depth_test = std::vector<bool>();
  return scene;
}

void add_render_func(Scene& scene, RenderFunc func, bool depth_test = true) {
  scene.render_funcs.push_back(func);
  scene.depth_test.push_back(depth_test);
}

void render_scene(Scene scene) {
  for (int i = 0; i < scene.render_funcs.size(); ++i) {
    if (scene.depth_test[i]) {
      glEnable(GL_DEPTH_TEST);
    } else {
      glDisable(GL_DEPTH_TEST);
    }
    scene.render_funcs[i](scene);
  }
}

struct InputHandler {
  static InputHandler& getInstance() {
    static InputHandler instance;
    return instance;
  }
  InputHandler(InputHandler const&) = delete;
  InputHandler(InputHandler&&) = delete;

  std::vector<MouseMoveFunc> mouse_move_funcs;
  std::vector<MouseInputFunc> mouse_input_funcs;
  std::vector<KeyInputFunc> key_input_funcs;
  float xpos = 0.0f;
  float ypos = 0.0f;
  bool left_pressing = false;

 private:
  InputHandler() {}
};

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

}  // namespace armgl

#endif  // GL_WARP_H_