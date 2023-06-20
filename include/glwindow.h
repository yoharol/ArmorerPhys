#ifndef GL_WINDOW_H_
#define GL_WINDOW_H_

#include <glad/glad.h>
#include <GLFW/glfw3.h>

namespace glrender {

GLFWwindow* create_window(int width, int height, const char* title) {
  GLFWwindow* window = glfwCreateWindow(width, height, title, NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    exit(-1);
  }
  glfwMakeContextCurrent(window);
  return window;
}

}  // namespace glrender

#endif  // GL_WINDOW_H_