#include "shader.h"
#include <fstream>
#include <string>

namespace {
std::string readFile(const std::filesystem::path& filename) {
  std::ifstream shaderFile(filename);
  if (!shaderFile) {
    std::string err = "Cannot open shader file: " + filename.string();
    if (glDebugMessageInsert) {
      glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_ERROR, 0, GL_DEBUG_SEVERITY_HIGH, -1,
                           err.c_str());
    } else {
      puts(err.c_str());
    }
  }
  auto shaderCode = std::string(std::istreambuf_iterator<char>(shaderFile), std::istreambuf_iterator<char>());
  return shaderCode;
}
}  // namespace

Shader::Shader(GLenum shaderType) noexcept : handle(glCreateShader(shaderType)) {}
Shader::~Shader() { glDeleteShader(handle); }

GLuint Shader::getHandle() const { return handle; }

bool Shader::checkCompileState() const {
  GLint success;
  GLchar infoLog[1024];
  glGetShaderiv(handle, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(handle, 1024, nullptr, infoLog);
    if (glDebugMessageInsert) {
      glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_ERROR, 0, GL_DEBUG_SEVERITY_HIGH, -1, infoLog);
    } else {
      puts("Shader compilation error!");
      puts(infoLog);
    }
  }
  return success;
}

void Shader::fromFile(const std::filesystem::path& filename) const { this->fromString(readFile(filename)); }

void Shader::fromString(const std::string& shaderCode) const {
  auto shaderCodePointer = shaderCode.c_str();
  glShaderSource(handle, 1, &shaderCodePointer, nullptr);
  glCompileShader(handle);
  checkCompileState();
}

GLuint ShaderProgram::currentBinding = 0;

ShaderProgram::ShaderProgram() noexcept : isLinked(false), handle(glCreateProgram()), uniforms(), uniformBlocks() {}

ShaderProgram::~ShaderProgram() { glDeleteProgram(handle); }

void ShaderProgram::attach(Shader* shader) { glAttachShader(handle, shader->getHandle()); }
void ShaderProgram::detach(Shader* shader) { glDetachShader(handle, shader->getHandle()); }

void ShaderProgram::link() {
  glLinkProgram(handle);
  isLinked = checkLinkState();
}

bool ShaderProgram::checkLinkState() const {
  GLint success;
  GLchar infoLog[1024];
  glGetProgramiv(handle, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(handle, 1024, nullptr, infoLog);
    if (glDebugMessageInsert) {
      glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_ERROR, 0, GL_DEBUG_SEVERITY_HIGH, -1, infoLog);
    } else {
      puts("Failed to link shader program!");
      puts(infoLog);
    }
  }
  return success;
}

GLuint ShaderProgram::getHandle() const { return handle; }

void ShaderProgram::use() const {
  // NOP if binds to the same shader program.
  if (handle == currentBinding) return;
  if (isLinked) {
    glUseProgram(handle);
    currentBinding = handle;
  }
}

GLint ShaderProgram::getUniformLocation(const char* name) const {
  // Cache the location since glGetUniformLocation calls are expensive
  auto it = uniforms.find(name);
  if (it != uniforms.end()) return it->second;
  GLint location = glGetUniformLocation(handle, name);
  uniforms.insert(std::make_pair(std::string(name), location));
  return location;
}

GLuint ShaderProgram::getUniformBlockIndex(const char* name) const {
  // Cache the location since glGetUniformBlockIndex calls are expensive
  auto it = uniformBlocks.find(name);
  if (it != uniformBlocks.end()) return it->second;
  GLint location = glGetUniformBlockIndex(handle, name);
  uniformBlocks.insert(std::make_pair(std::string(name), location));
  return location;
}

void ShaderProgram::uniformBlockBinding(const char* name, GLuint uniformBlockBinding) const {
  GLuint blockindex = getUniformBlockIndex(name);
  if (blockindex != GL_INVALID_INDEX) glUniformBlockBinding(handle, blockindex, uniformBlockBinding);
}

void ShaderProgram::uniformBlockBinding(GLuint uniformBlockIndex, GLuint uniformBlockBinding) const {
  if (uniformBlockIndex != GL_INVALID_INDEX) glUniformBlockBinding(handle, uniformBlockIndex, uniformBlockBinding);
}

void ShaderProgram::setUniform(const char* name, GLint i1) { glUniform1i(getUniformLocation(name), i1); }
void ShaderProgram::setUniform(const char* name, GLfloat f1) { glUniform1f(getUniformLocation(name), f1); }
void ShaderProgram::setUniform(const char* name, const Eigen::Matrix4f& mat4) {
  glUniformMatrix4fv(getUniformLocation(name), 1, GL_FALSE, mat4.data());
}
void ShaderProgram::setUniform(const char* name, const Eigen::Vector4f& vec4) {
  glUniform4fv(getUniformLocation(name), 1, vec4.data());
}
