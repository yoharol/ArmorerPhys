#ifndef GL_IMGUI_H_
#define GL_IMGUI_H_

#include <functional>
#include <string>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace aphys {

typedef std::function<void()> GuiFunc;
typedef std::function<void()> GuiMouseInputFunc;
typedef std::function<void()> GuiKeyInputFunc;

struct Gui {
  std::string name;
  ImGuiIO &io;
  bool open;
  std::vector<GuiFunc> gui_func;
  std::vector<GuiMouseInputFunc> gui_mouse_input_func;
  std::vector<GuiKeyInputFunc> gui_key_input_func;
  int width;
  int height;
  int window_width;
  int window_height;
};

Gui create_gui(GLFWwindow *window, std::string name,
               const char *glsl_version = "#version 330");

void add_gui_func(Gui &imgui_window, GuiFunc gui_func);

void add_gui_mouse_input_func(Gui &imgui_window, GuiMouseInputFunc func);

void add_gui_key_input_func(Gui &imgui_window, GuiKeyInputFunc func);

void handle_gui_input(Gui &imgui_window);

void input_process_gui(Gui &imgui_window);

void render_gui(Gui &imgui_window);

void destroy_gui(Gui &imgui_window);

void StyleColorsSpectrum();

}  // namespace aphys

#endif  // GL_IMGUI_H_