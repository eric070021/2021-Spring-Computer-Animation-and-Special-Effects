#pragma once
#include <GLFW/glfw3.h>
#include <glad/gl.h>

#include "utils.h"

class OpenGLContext final {
 public:
  // Not copyable
  DELETE_COPY(OpenGLContext)
  // Not movable
  DELETE_MOVE(OpenGLContext)
  /// @brief Release resources
  ~OpenGLContext();
  /// @brief Get OpenGL context.
  static OpenGLContext& getContext();
  /// @return Current window handle.
  GLFWwindow* createWindow(const char* name, int width, int height, GLenum profile);
  /// @return The monitor refresh rate.
  int getRefreshRate() const { return refreshRate; }
  /// @return The OpenGL context version.
  int getOpenGLVersion() const { return majorVersion * 10 + minorVersion; }
  /// @brief Print the system information.
  void printSystemInfo() const;
  /// @brief Enable OpenGL's debug callback, useful for debugging.
  void enableDebugCallback();

 private:
  /// @brief Create OpenGL context, call by createContext method
  OpenGLContext();
  int majorVersion, minorVersion;
  int profile;
  int refreshRate;
};
