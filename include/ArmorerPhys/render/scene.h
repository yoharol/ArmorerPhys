#ifndef GL_WARP_H_
#define GL_WARP_H_

#include <functional>

#include "ArmorerPhys/render/shader.h"
#include "ArmorerPhys/render/camera.h"
#include "ArmorerPhys/render/scene.h"

namespace aphys {

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
  float delta_time;
  float time;
};

Scene create_scene(Light light, Camera camera);

void add_render_func(Scene& scene, RenderFunc func, bool depth_test = true);

void render_scene(Scene& scene);

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

InputHandler& create_input_handler(GLFWwindow* window);

void add_mouse_move_func(InputHandler& handler, MouseMoveFunc func);

void add_mouse_input_func(InputHandler& handler, MouseInputFunc func);

void add_key_input_func(InputHandler& handler, KeyInputFunc func);

}  // namespace aphys

#endif  // GL_WARP_H_