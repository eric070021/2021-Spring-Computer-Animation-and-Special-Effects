#include "gui.h"
#include <cmath>

#include "configs.h"
#include "icons.h"

namespace {
void renderMainPanel() {
  ImGui::SetNextWindowSize(ImVec2(380.0f, 100.0f), ImGuiCond_Once);
  ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
  ImGui::SetNextWindowPos(ImVec2(160.0f, 600.0f), ImGuiCond_Once);
  ImGui::SetNextWindowBgAlpha(0.2f);

  if (ImGui::Begin(ICON_CODE " Frame Control")) {
    int upperBound = std::max(0, maxFrame - 1);
    ImGui::SliderInt("Current frame", &currentFrame, 0, upperBound);
    if (ImGui::Button(ICON_PLAY)) {
      isSimulating = true;
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_PAUSE)) {
      isSimulating = false;
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_STOP)) {
      isSimulating = false;
      currentFrame = 0;
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_MINUS)) {
      currentFrame = std::max(0, currentFrame - 1);
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_PLUS)) {
      currentFrame = std::min(upperBound, currentFrame + 1);
    }
    isMotionChanged |= ImGui::RadioButton("punch", &currentMotion, 0);
    ImGui::SameLine();
    isMotionChanged |= ImGui::RadioButton("walk_run", &currentMotion, 1);
  }
  ImGui::End();
}

void renderCameraPanel() {
  ImGui::SetNextWindowSize(ImVec2(460.0f, 100.0f), ImGuiCond_Once);
  ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
  ImGui::SetNextWindowPos(ImVec2(630.0f, 600.0f), ImGuiCond_Once);
  ImGui::SetNextWindowBgAlpha(0.2f);
  if (ImGui::Begin(ICON_EYE " Camera")) {
    if (ImGui::InputFloat("mouseMoveSpeed", &mouseMoveSpeed, 1e-4f, 1e-3f, "%.4f")) {
      mouseMoveSpeed = std::max(0.0f, mouseMoveSpeed);
    }
    if (ImGui::InputFloat("keyboardMoveSpeed", &keyboardMoveSpeed, 1e-2f, 1e-1f, "%.2f")) {
      keyboardMoveSpeed = std::max(0.0f, keyboardMoveSpeed);
    }
    ImGui::Text("Current framerate: %.0f", ImGui::GetIO().Framerate);
  }
  ImGui::End();
}
}  // namespace

GUI::GUI(GLFWwindow* window, int version) {
  static constexpr const ImWchar icon_ranges[] = {ICON_MIN, ICON_MAX, 0};
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  if (version >= 41) {
    ImGui_ImplOpenGL3_Init("#version 410 core");
  } else {
    ImGui_ImplOpenGL3_Init(nullptr);
  }
  ImGuiIO& io = ImGui::GetIO();
  ImFontConfig config;
  config.GlyphMaxAdvanceX = 8.0f;
  config.GlyphMinAdvanceX = 8.0f;
  config.OversampleH = 1;
  config.OversampleV = 1;
  config.PixelSnapH = true;
  io.Fonts->AddFontDefault(&config);
  config.MergeMode = true;
  config.FontDataOwnedByAtlas = false;
  io.Fonts->AddFontFromMemoryTTF((void*)forkawesome, sizeof(forkawesome), 13.0f, &config, icon_ranges);
  io.Fonts->Build();
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
  renderCameraPanel();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
