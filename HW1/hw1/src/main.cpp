#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>
#define GLAD_GL_IMPLEMENTATION
#include <glad/gl.h>
#undef GLAD_GL_IMPLEMENTATION

#include "hw1.h"
int alignSize = 256;
bool isWindowSizeChanged = true;
bool mouseBinded = false;

int uboAlign(int i) { return ((i + 1 * (alignSize - 1)) / alignSize) * alignSize; }

void keyCallback(GLFWwindow* window, int key, int, int action, int) {
  // There are three actions: press, release, hold
  if (action != GLFW_PRESS) return;
  // Press ESC to close the window.
  if (key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  } else if (key == GLFW_KEY_F9) {
    // Disable / enable mouse cursor.
    if (mouseBinded)
      glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    else
      glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    mouseBinded = !mouseBinded;
  }
}

void framebufferSizeCallback(GLFWwindow*, int width, int height) {
  // Minimize event guard
  if (width == 0 && height == 0) return;
  windowWidth = width;
  windowHeight = height;
  glViewport(0, 0, width, height);
  isWindowSizeChanged = true;
}

int main() {
  // Initialize OpenGL context.
  OpenGLContext& context = OpenGLContext::getContext();
  GLFWwindow* window = context.createWindow("HW1", 1280, 720, GLFW_OPENGL_CORE_PROFILE);
  glfwSetKeyCallback(window, keyCallback);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

#ifndef NDEBUG
  context.printSystemInfo();
  context.enableDebugCallback();
#endif

  glfwSwapInterval(1);
  glfwGetFramebufferSize(window, &windowWidth, &windowHeight);
  glViewport(0, 0, windowWidth, windowHeight);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClearColor(0, 0, 0, 1);
  glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &alignSize);

  speedMultiplier = (240 / context.getRefreshRate());
  simulationPerFrame *= speedMultiplier;
  GUI gui(window, context.getOpenGLVersion());
  // Initialize shaders
  ShaderProgram sphereRenderer, particleRenderer;
  {
    VertexShader vs;
    FragmentShader fs;
    vs.fromFile(findPath("sphere.vert"));
    fs.fromFile(findPath("sphere.frag"));
    sphereRenderer.attach(&vs, &fs);
    sphereRenderer.link();
    sphereRenderer.detach(&vs, &fs);
    sphereRenderer.use();
    sphereRenderer.setUniform("color", sphereColor);
    sphereRenderer.uniformBlockBinding("model", 0);
    sphereRenderer.uniformBlockBinding("camera", 1);
  }
  {
    VertexShader vs;
    FragmentShader fs;
    vs.fromFile(findPath("particle.vert"));
    fs.fromFile(findPath("particle.frag"));
    particleRenderer.attach(&vs, &fs);
    particleRenderer.link();
    particleRenderer.detach(&vs, &fs);
    particleRenderer.use();

    particleRenderer.setUniform("color", clothColor);
    particleRenderer.uniformBlockBinding("model", 0);
    particleRenderer.uniformBlockBinding("camera", 1);
  }
  // Create softbody
  Cloth cloth;
  cloth.computeNormal();
  UniformBuffer meshUBO;
  int meshOffset = uboAlign(32 * sizeof(GLfloat));
  meshUBO.allocate(2 * meshOffset);
  meshUBO.load(0, 16 * sizeof(GLfloat), cloth.getModelMatrix().data());
  meshUBO.load(16 * sizeof(GLfloat), 16 * sizeof(GLfloat), cloth.getNormalMatrix().data());

  Spheres& spheres = Spheres::initSpheres();
  spheres.addSphere(Eigen::Vector4f(-0.75, 1, -0.75, 1), 0.5f);
  spheres.addSphere(Eigen::Vector4f(0.75, 1, -0.75, 1), 0.5f);
  spheres.addSphere(Eigen::Vector4f(-0.75, 1, 0.75, 1), 0.5f);
  spheres.addSphere(Eigen::Vector4f(0.75, 1, 0.75, 1), 0.5f);
  meshUBO.load(meshOffset, 16 * sizeof(GLfloat), spheres.getModelMatrix().data());
  meshUBO.load(meshOffset + 16 * sizeof(GLfloat), 16 * sizeof(GLfloat), spheres.getNormalMatrix().data());

  Camera camera(Eigen::Vector4f(0, 2, -10, 1));
  UniformBuffer cameraUBO;
  cameraUBO.allocate(uboAlign(20 * sizeof(GLfloat)));
  cameraUBO.load(0, 16 * sizeof(GLfloat), camera.viewProjectionMatrix().data());
  cameraUBO.load(16 * sizeof(GLfloat), 4 * sizeof(GLfloat), camera.position().data());
  cameraUBO.bindUniformBlockIndex(1, 0, uboAlign(20 * sizeof(GLfloat)));
  // Do one step simulation, used in some implicit methods
  std::function<void(void)> simulateOneStep = [&]() {
    cloth.computeExternalForce();
    spheres.computeExternalForce();
    cloth.computeSpringForce();
    spheres.collide(&cloth);
    spheres.collide();
  };

  ExplicitEuler explicitEuler;
  ImplicitEuler implicitEuler;
  MidpointEuler midpointEuler;
  RungeKuttaFourth rk4;
  Integrator* integrator = &explicitEuler;

  std::vector<Particles*> particles{&cloth.particles(), &spheres.particles()};
  // Backup initial state
  Particles initialCloth = cloth.particles();
  Particles initialSpheres = spheres.particles();

  while (!glfwWindowShouldClose(window)) {
    // Polling events.
    glfwPollEvents();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    bool cameraChanged = mouseBinded ? camera.move(window) : false;
    if (isWindowSizeChanged) {
      isWindowSizeChanged = false;
      camera.updateProjection();
      cameraChanged = true;
    }
    if (cameraChanged) {
      cameraUBO.load(0, 16 * sizeof(GLfloat), camera.viewProjectionMatrix().data());
      cameraUBO.load(16 * sizeof(GLfloat), 4 * sizeof(GLfloat), camera.position().data());
    }
    // Check which integrator is selected in GUI.
    switch (currentIntegrator) {
      case 0: integrator = &explicitEuler; break;
      case 1: integrator = &implicitEuler; break;
      case 2: integrator = &midpointEuler; break;
      case 3: integrator = &rk4; break;
      default: break;
    }

    if (!isPaused) {
      // Stop -> Start: Restore initial state
      if (isStateSwitched) {
        cloth.particles() = initialCloth;
        spheres.particles() = initialSpheres;
      }
      // Simulate one step and then integrate it.
      for (int i = 0; i < simulationPerFrame; i++) {
        simulateOneStep();
        integrator->integrate(particles, simulateOneStep);
      }
    }

    particleRenderer.use();
    if (isClothColorChange) particleRenderer.setUniform("color", clothColor);
    meshUBO.bindUniformBlockIndex(0, 0, meshOffset);
    if (isDrawingParticles) cloth.draw(Cloth::DrawType::PARTICLE);
    if (isDrawingStructuralSprings) cloth.draw(Cloth::DrawType::STRUCTURAL);
    if (isDrawingShearSprings) cloth.draw(Cloth::DrawType::SHEAR);
    if (isDrawingBendSprings) cloth.draw(Cloth::DrawType::BEND);
    if (isDrawingCloth) {
      glDisable(GL_CULL_FACE);
      // This is very slow because it is done in CPU. Since GL4.1 doesn't support compute shader.
      cloth.computeNormal();
      particleRenderer.setUniform("isSurface", 1);
      cloth.draw(Cloth::DrawType::FULL);
      glEnable(GL_CULL_FACE);
    } else {
      particleRenderer.setUniform("isSurface", 0);
    }
    sphereRenderer.use();
    if (isSphereColorChange) sphereRenderer.setUniform("color", sphereColor);
    meshUBO.bindUniformBlockIndex(0, meshOffset, meshOffset);
    spheres.draw();

    gui.render();
#ifdef __APPLE__
    glFlush();
#endif
    glfwSwapBuffers(window);
  }
  glfwDestroyWindow(window);
  return 0;
}
