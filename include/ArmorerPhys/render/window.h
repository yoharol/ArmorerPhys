#ifndef GL_WINDOW_H_
#define GL_WINDOW_H_

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "ArmorerPhys/type.h"

namespace aphys {

struct Window {
  static Window& get_instance() {
    static Window instance;
    return instance;
  }
  GLFWwindow* get_window() { return window; }
  void set_window(GLFWwindow* window) { this->window = window; }
  int width;
  int height;
  Window(Window const&) = delete;
  void operator=(Window const&) = delete;

 private:
  GLFWwindow* window;

  Window() : window(nullptr), width(0), height(0) {}
};

void GLFW_error(int error, const char* description);

void init_glfw();

void init_glad();

GLFWwindow* create_window(int width, int height, const char* title);

void set_blend_transparent();

void set_wireframe_mode(bool wireframe);

void set_background_RGB(RGB color);

}  // namespace aphys

#endif  // GL_WINDOW_H_