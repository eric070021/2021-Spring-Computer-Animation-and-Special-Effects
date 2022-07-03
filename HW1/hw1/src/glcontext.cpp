#include "glcontext.h"

#include <cstring>
#include <iomanip>
#include <iostream>
namespace {

#if defined(GL_VERSION_4_3) || defined(GL_KHR_debug)
void printSourceEnum(GLenum source) {
  std::cerr << "Source  : ";
  switch (source) {
    case GL_DEBUG_SOURCE_API: std::cerr << "API"; break;
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM: std::cerr << "Window system"; break;
    case GL_DEBUG_SOURCE_SHADER_COMPILER: std::cerr << "Shader compiler"; break;
    case GL_DEBUG_SOURCE_THIRD_PARTY: std::cerr << "Third party"; break;
    case GL_DEBUG_SOURCE_APPLICATION: std::cerr << "Application"; break;
    case GL_DEBUG_SOURCE_OTHER: [[fallthrough]];
    default: std::cerr << "Other"; break;
  }
  std::cerr << std::endl;
}

void printTypeEnum(GLenum type) {
  std::cerr << "Type    : ";
  switch (type) {
    case GL_DEBUG_TYPE_ERROR: std::cerr << "Error"; break;
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: std::cerr << "Deprecated behavior"; break;
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR: std::cerr << "Undefined behavior"; break;
    case GL_DEBUG_TYPE_PORTABILITY: std::cerr << "Portability"; break;
    case GL_DEBUG_TYPE_PERFORMANCE: std::cerr << "Performance"; break;
    case GL_DEBUG_TYPE_MARKER: std::cerr << "Marker"; break;
    case GL_DEBUG_TYPE_PUSH_GROUP: std::cerr << "Push group"; break;
    case GL_DEBUG_TYPE_POP_GROUP: std::cerr << "Pop group"; break;
    case GL_DEBUG_TYPE_OTHER: [[fallthrough]];
    default: std::cerr << "Other"; break;
  }
  std::cerr << std::endl;
}

void printSeverityEnum(GLenum severity) {
  std::cerr << "Severity: ";
  switch (severity) {
    case GL_DEBUG_SEVERITY_HIGH: std::cerr << "High"; break;
    case GL_DEBUG_SEVERITY_MEDIUM: std::cerr << "Medium"; break;
    case GL_DEBUG_SEVERITY_LOW: std::cerr << "Low"; break;
    case GL_DEBUG_SEVERITY_NOTIFICATION: [[fallthrough]];
    default: std::cerr << "Notification"; break;
  }
  std::cerr << std::endl;
}

void GLAPIENTRY
errorCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei, const GLchar* message, const void*) {
  if (id == 131169 ||  // Allocate framebuffer
      id == 131185 ||  // Allocate buffer
      id == 131218 ||  // Shader recompile
      id == 131204     // Texture no base level
  )
    return;
  std::cerr << std::endl << "Message : " << message << std::endl;
  printSeverityEnum(severity);
  printSourceEnum(source);
  printTypeEnum(type);
}
#endif
}  // namespace

OpenGLContext::OpenGLContext() {
  // Initialize GLFW
  if (glfwInit() == GLFW_FALSE) {
    std::cerr << "GLFW initialization failed" << std::endl;
    exit(EXIT_FAILURE);
  }
#ifndef __APPLE__
  // Create a dummy window to test OpenGL version
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* dummyWindow = glfwCreateWindow(1, 1, "dummy", nullptr, nullptr);
  glfwMakeContextCurrent(dummyWindow);
  int version = gladLoadGL(glfwGetProcAddress);
  if (version == 0) {
    std::cerr << "GLAD initialization failed" << std::endl;
  }
  OpenGLContext::majorVersion = GLAD_VERSION_MAJOR(version);
  OpenGLContext::minorVersion = GLAD_VERSION_MINOR(version);
  // Free the dummyWindow
  glfwDestroyWindow(dummyWindow);
  GladGLContext* context = gladGetGLContext();
  memset(context, 0, sizeof(GladGLContext));
#else
  OpenGLContext::majorVersion = 3;
  OpenGLContext::minorVersion = 3;
#endif
  GLFWmonitor* moniter = glfwGetPrimaryMonitor();
  const GLFWvidmode* vidMode = glfwGetVideoMode(moniter);
  if (vidMode == nullptr) {
    std::cerr << "Unable to get video mode of monitor." << std::endl;
    return;
  }
  OpenGLContext::refreshRate = vidMode->refreshRate;
}

OpenGLContext::~OpenGLContext() { glfwTerminate(); }

GLFWwindow* OpenGLContext::createWindow(const char* name, int width, int height, GLenum profile) {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, OpenGLContext::majorVersion);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, OpenGLContext::minorVersion);
  if (OpenGLContext::majorVersion * 10 + OpenGLContext::minorVersion <= 32) {
    profile = GLFW_OPENGL_ANY_PROFILE;
  }
  if (profile == GLFW_OPENGL_CORE_PROFILE) {
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  }
  glfwWindowHint(GLFW_OPENGL_PROFILE, profile);
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
#ifndef NDEBUG
  glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
#endif
  GLFWwindow* window = glfwCreateWindow(width, height, name, nullptr, nullptr);
  glfwMakeContextCurrent(window);
  gladLoadGL(glfwGetProcAddress);
  return window;
}

OpenGLContext& OpenGLContext::getContext() {
  static OpenGLContext context;
  return context;
}

void OpenGLContext::printSystemInfo() const {
  std::cout << std::left << std::setw(26) << "Current OpenGL renderer"
            << ": " << glGetString(GL_RENDERER) << std::endl;
  std::cout << std::left << std::setw(26) << "Current OpenGL context"
            << ": " << glGetString(GL_VERSION) << std::endl;
  std::cout << std::left << std::setw(26) << "Moniter refresh rate"
            << ": " << refreshRate << " Hz" << std::endl;
}

void OpenGLContext::enableDebugCallback() {
#if defined(GL_VERSION_4_3) || defined(GL_KHR_debug)
  if (glDebugMessageCallback != nullptr) {
    std::cout << "Debug context enabled." << std::endl;
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(errorCallback, nullptr);
  } else {
    std::cout << "Your system does not support debug output." << std::endl;
    std::cout << "Your can manually use glGetError to debug." << std::endl;
  }
#endif
}
