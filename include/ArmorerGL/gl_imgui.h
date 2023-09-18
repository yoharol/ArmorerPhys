#ifndef GL_IMGUI_H_
#define GL_IMGUI_H_

#include <functional>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace armgl {

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
};

void StyleColorsSpectrum();

Gui create_gui(GLFWwindow *window, std::string name,
               const char *glsl_version = "#version 330") {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  ImFontConfig config;
  config.OversampleH = 3;
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();
  StyleColorsSpectrum();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);
  Gui imgui_window{name, io, true, {}, {}, {}, 300, 400};
  imgui_window.io = io;
  imgui_window.open = true;
  return imgui_window;
}

void add_gui_func(Gui &imgui_window, GuiFunc gui_func) {
  imgui_window.gui_func.push_back(gui_func);
}

void add_gui_mouse_input_func(Gui &imgui_window, GuiMouseInputFunc func) {
  imgui_window.gui_mouse_input_func.push_back(func);
}

void add_gui_key_input_func(Gui &imgui_window, GuiKeyInputFunc func) {
  imgui_window.gui_key_input_func.push_back(func);
}

void handle_gui_input(Gui &imgui_window) {
  bool imguiCapturingMouse = ImGui::GetIO().WantCaptureMouse;
  if (!imguiCapturingMouse) {
    for (auto &func : imgui_window.gui_mouse_input_func) {
      func();
    }
  }
  bool imguiCapturingKeyboard = ImGui::GetIO().WantCaptureKeyboard;
  if (!imguiCapturingKeyboard) {
    for (auto &func : imgui_window.gui_key_input_func) {
      func();
    }
  }
}

void input_process_gui(Gui &imgui_window) { handle_gui_input(imgui_window); }

void render_gui(Gui &imgui_window) {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
  ImGui::SetNextWindowSize(ImVec2(imgui_window.width, imgui_window.height),
                           ImGuiCond_Once);
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

void StyleColorsSpectrum() {
  ImGuiStyle &style = ImGui::GetStyle();
  style.WindowMinSize = ImVec2(160, 20);
  style.FramePadding = ImVec2(4, 2);
  style.ItemSpacing = ImVec2(6, 6);
  style.ItemInnerSpacing = ImVec2(6, 4);
  style.Alpha = 0.95f;
  style.WindowRounding = 4.0f;
  style.FrameRounding = 2.0f;
  style.IndentSpacing = 6.0f;
  style.ItemInnerSpacing = ImVec2(2, 4);
  style.ColumnsMinSpacing = 50.0f;
  style.GrabMinSize = 14.0f;
  style.GrabRounding = 16.0f;
  style.ScrollbarSize = 12.0f;
  style.ScrollbarRounding = 16.0f;

  style.Colors[ImGuiCol_Text] = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
  style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
  style.Colors[ImGuiCol_WindowBg] = ImVec4(0.94f, 0.94f, 0.94f, 1.00f);
  style.Colors[ImGuiCol_Border] = ImVec4(0.00f, 0.00f, 0.00f, 0.39f);
  style.Colors[ImGuiCol_BorderShadow] = ImVec4(1.00f, 1.00f, 1.00f, 0.10f);
  style.Colors[ImGuiCol_FrameBg] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
  style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
  style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
  style.Colors[ImGuiCol_TitleBg] = ImVec4(0.96f, 0.96f, 0.96f, 1.00f);
  style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(1.00f, 1.00f, 1.00f, 0.51f);
  style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.82f, 0.82f, 0.82f, 1.00f);
  style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
  style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.98f, 0.98f, 0.98f, 0.53f);
  style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.69f, 0.69f, 0.69f, 0.80f);
  style.Colors[ImGuiCol_ScrollbarGrabHovered] =
      ImVec4(0.49f, 0.49f, 0.49f, 0.80f);
  style.Colors[ImGuiCol_ScrollbarGrabActive] =
      ImVec4(0.49f, 0.49f, 0.49f, 1.00f);
  style.Colors[ImGuiCol_CheckMark] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.26f, 0.59f, 0.98f, 0.78f);
  style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  style.Colors[ImGuiCol_Button] = ImVec4(0.76f, 0.76f, 0.76f, 1.00f);
  style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.36f, 0.53f, 0.58f, 1.00f);
  style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.36f, 0.53f, 0.58f, 1.00f);
  style.Colors[ImGuiCol_Header] = ImVec4(0.26f, 0.59f, 0.98f, 0.31f);
  style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.80f);
  style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  style.Colors[ImGuiCol_ResizeGrip] = ImVec4(1.00f, 1.00f, 1.00f, 0.00f);
  style.Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
  style.Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
  ImVec4(0.98f, 0.39f, 0.36f, 1.00f);
  style.Colors[ImGuiCol_PlotLines] = ImVec4(0.39f, 0.39f, 0.39f, 1.00f);
  style.Colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
  style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
  style.Colors[ImGuiCol_PlotHistogramHovered] =
      ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
  style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.26f, 0.59f, 0.98f, 0.35f);
}

}  // namespace armgl

#endif  // GL_IMGUI_H_