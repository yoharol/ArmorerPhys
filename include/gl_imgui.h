#ifndef GL_IMGUI_H_
#define GL_IMGUI_H_

#include <functional>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace glrender {

typedef std::function<void()> GuiFunc;

struct Gui {
  std::string name;
  ImGuiIO &io;
  bool open;
  std::vector<GuiFunc> gui_func;
};

Gui create_gui(GLFWwindow *window, std::string name,
               const char *glsl_version = "#version 120") {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);
  Gui imgui_window{name, io, true, {}};
  imgui_window.io = io;
  imgui_window.open = true;
  return imgui_window;
}

void add_gui_func(Gui &imgui_window, GuiFunc gui_func) {
  imgui_window.gui_func.push_back(gui_func);
}

void render_gui(Gui &imgui_window) {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGui::Begin(imgui_window.name.c_str(), &imgui_window.open);
  if (imgui_window.open) {
    for (auto &gui_func : imgui_window.gui_func) {
      gui_func();
    }
  }
  ImGui::End();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void destroy_gui(Gui &imgui_window) {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

}  // namespace glrender

#endif  // GL_IMGUI_H_