#ifndef GL_WINDOW_H_
#define GL_WINDOW_H_

#include <glad/glad.h>
#include <GLFW/glfw3.h>

namespace armgl {

void GLFW_error(int error, const char* description) {
  fputs(description, stderr);
}

void init_glfw() {
  glfwSetErrorCallback(GLFW_error);
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
}

void init_glad() {
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    exit(-1);
  }
  glEnable(GL_DEPTH_TEST);
  // enable transparent blend
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

GLFWwindow* create_window(int width, int height, const char* title) {
  init_glfw();
  GLFWwindow* window = glfwCreateWindow(width, height, title, NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    exit(-1);
  }
  glfwMakeContextCurrent(window);
  init_glad();
  return window;
}

void set_blend_transparent() {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void set_wireframe_mode(bool wireframe) {
  if (wireframe) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  } else {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
}

void set_background_RGB(RGB color) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(color(0) / 255.0f, color(1) / 255.0f, color(2) / 255.0f, 1.0f);
}

}  // namespace armgl

#endif  // GL_WINDOW_H_