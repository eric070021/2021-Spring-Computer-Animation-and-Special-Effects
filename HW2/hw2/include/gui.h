#pragma once
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "utils.h"

class GUI {
 public:
  explicit GUI(GLFWwindow* window, int version = 0);
  ~GUI();
  DELETE_COPY(GUI)
  DELETE_MOVE(GUI)
  /**
   * @brief Render the GUI panel.
   *
   */
  void render();
};
