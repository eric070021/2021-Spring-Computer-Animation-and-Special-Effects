#include "gui.h"
#include <cmath>

#include "configs.h"

namespace {
void renderColorPanel() {
  static bool isUsingSphereColor = false;
  static bool isUsingClothColor = false;

  if (ImGui::Button("Change sphere color")) isUsingSphereColor = !isUsingSphereColor;
  ImGui::SameLine();
  if (ImGui::Button("Change cloth color")) isUsingClothColor = !isUsingClothColor;
  if (isUsingSphereColor) {
    ImGui::SetNextWindowSize(ImVec2(300.0f, 300.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(50.0f, 50.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    if (ImGui::Begin("Sphere color", &isUsingSphereColor)) {
      isSphereColorChange = ImGui::ColorPicker4("Sphere Color", sphereColor.data());
    }
    ImGui::End();
  }

  if (isUsingClothColor) {
    ImGui::SetNextWindowSize(ImVec2(300.0f, 300.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(50.0f, 50.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    if (ImGui::Begin("Cloth color", &isUsingClothColor)) {
      isClothColorChange = ImGui::ColorPicker4("Cloth Color", clothColor.data());
    }
    ImGui::End();
  }
}

void renderDrawingTypes() {
  ImGui::Checkbox("Particles", &isDrawingParticles);
  ImGui::SameLine();
  ImGui::Checkbox("Structural", &isDrawingStructuralSprings);
  ImGui::SameLine();
  ImGui::Checkbox("Shear", &isDrawingShearSprings);
  ImGui::SameLine();
  ImGui::Checkbox("Bend", &isDrawingBendSprings);
  ImGui::SameLine();
  ImGui::Checkbox("Surface", &isDrawingCloth);
}

void renderMainPanel() {
  ImGui::SetNextWindowSize(ImVec2(450.0f, 350.0f), ImGuiCond_Once);
  ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
  ImGui::SetNextWindowPos(ImVec2(50.0f, 50.0f), ImGuiCond_Once);
  ImGui::SetNextWindowBgAlpha(0.2f);
  if (ImGui::Begin("Configs")) {
    if (ImGui::InputFloat("mouseMoveSpeed", &mouseMoveSpeed, 1e-4f, 1e-3f, "%.4f")) {
      mouseMoveSpeed = std::max(0.0f, mouseMoveSpeed);
    }
    if (ImGui::InputFloat("keyboardMoveSpeed", &keyboardMoveSpeed, 1e-2f, 1e-1f, "%.2f")) {
      keyboardMoveSpeed = std::max(0.0f, keyboardMoveSpeed);
    }
    if (ImGui::InputFloat("deltaTime", &deltaTime, 1e-4f, 1e-5f, "%.5f")) {
      deltaTime = std::max(0.0f, deltaTime);
      simulationPerFrame = speedMultiplier * static_cast<int>(baseSpeed / deltaTime);
      simulationPerFrame = std::max(1, simulationPerFrame);
    }
    if (ImGui::InputFloat("springCoef", &springCoef, 1e2f, 1e3f, "%.0f")) {
      springCoef = std::max(0.0f, springCoef);
    }
    if (ImGui::InputFloat("damperCoef", &damperCoef, 1.0f, 1e2f, "%.0f")) {
      damperCoef = std::max(0.0f, damperCoef);
    }

    ImGui::Text("%s", "---------------------- Integrator ----------------------");
    ImGui::RadioButton("Explicit Euler", &currentIntegrator, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Implicit Euler", &currentIntegrator, 1);
    ImGui::RadioButton("Midpoint Euler", &currentIntegrator, 2);
    ImGui::SameLine();
    ImGui::RadioButton("Runge Kutta Fourth", &currentIntegrator, 3);

    ImGui::Text("%s", "-------------------- Drawing Config --------------------");
    renderColorPanel();
    renderDrawingTypes();
    ImGui::Text("%s", "-------------------- Miscellaneous ---------------------");
    if ((isStateSwitched = ImGui::Button(isPaused ? "Start" : "Stop"))) isPaused = !isPaused;
    ImGui::Text("Current framerate: %.0f", ImGui::GetIO().Framerate);
  }
  ImGui::End();
}
}  // namespace

GUI::GUI(GLFWwindow* window, int version) {
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  if (version >= 41) {
    ImGui_ImplOpenGL3_Init("#version 410 core");
  } else {
    ImGui_ImplOpenGL3_Init(nullptr);
  }
}

GUI::~GUI() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

void GUI::render() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  renderMainPanel();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
