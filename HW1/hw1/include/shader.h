#pragma once
#include <string>
#include <unordered_map>

#include <glad/gl.h>
#include <Eigen/Core>

#include "utils.h"

class Shader {
 public:
  MOVE_ONLY(Shader)
  virtual ~Shader();
  CONSTEXPR_VIRTUAL virtual const char* getTypeName() const = 0;
  CONSTEXPR_VIRTUAL virtual GLenum getType() const = 0;
  GLuint getHandle() const;
  bool checkCompileState() const;
  void fromFile(const std::filesystem::path& filename) const;
  void fromString(const std::string& shadercode) const;

 protected:
  explicit Shader(GLenum shaderType) noexcept;
  GLuint handle;
};

class ComputeShader final : public Shader {
 public:
  ComputeShader() noexcept : Shader(GL_COMPUTE_SHADER) {}
  CONSTEXPR_VIRTUAL const char* getTypeName() const override { return "Compute shader"; }
  CONSTEXPR_VIRTUAL GLenum getType() const override { return GL_COMPUTE_SHADER; }
};

class VertexShader final : public Shader {
 public:
  VertexShader() noexcept : Shader(GL_VERTEX_SHADER) {}
  CONSTEXPR_VIRTUAL const char* getTypeName() const override { return "Vertex shader"; }
  CONSTEXPR_VIRTUAL GLenum getType() const override { return GL_VERTEX_SHADER; }
};

class TessControlShader final : public Shader {
 public:
  TessControlShader() noexcept : Shader(GL_TESS_CONTROL_SHADER) {}
  CONSTEXPR_VIRTUAL const char* getTypeName() const override { return "Tessellation control shader"; }
  CONSTEXPR_VIRTUAL GLenum getType() const override { return GL_TESS_CONTROL_SHADER; }
};

class TessEvaluationShader final : public Shader {
 public:
  TessEvaluationShader() noexcept : Shader(GL_TESS_EVALUATION_SHADER) {}
  CONSTEXPR_VIRTUAL const char* getTypeName() const override { return "Tessellation evaluation shader"; }
  CONSTEXPR_VIRTUAL GLenum getType() const override { return GL_TESS_EVALUATION_SHADER; }
};

class GeometryShader final : public Shader {
 public:
  GeometryShader() noexcept : Shader(GL_GEOMETRY_SHADER) {}
  CONSTEXPR_VIRTUAL const char* getTypeName() const override { return "Geometry shader"; }
  CONSTEXPR_VIRTUAL GLenum getType() const override { return GL_GEOMETRY_SHADER; }
};

class FragmentShader final : public Shader {
 public:
  MOVE_ONLY(FragmentShader)
  FragmentShader() noexcept : Shader(GL_FRAGMENT_SHADER) {}
  CONSTEXPR_VIRTUAL const char* getTypeName() const override { return "Fragment shader"; }
  CONSTEXPR_VIRTUAL GLenum getType() const override { return GL_FRAGMENT_SHADER; }
};

class ShaderProgram final {
 public:
  MOVE_ONLY(ShaderProgram)
  ShaderProgram() noexcept;
  ~ShaderProgram();
  void attach(Shader* shader);

  template <class... Shaders>
  void attach(Shader* shader, Shaders... shaders) {
    attach(shader);
    attach(std::forward<Shaders>(shaders)...);
  }

  void detach(Shader* shader);

  template <class... Shaders>
  void detach(Shader* shader, Shaders... shaders) {
    detach(shader);
    detach(std::forward<Shaders>(shaders)...);
  }

  void link();
  bool checkLinkState() const;

  GLuint getHandle() const;
  void use() const;

  GLint getUniformLocation(const char* name) const;
  GLuint getUniformBlockIndex(const char* name) const;
  void uniformBlockBinding(const char* name, GLuint binding) const;
  void uniformBlockBinding(GLuint index, GLuint binding) const;

  void setUniform(const char* name, GLint i1);
  void setUniform(const char* name, GLfloat f1);
  void setUniform(const char* name, const Eigen::Matrix4f& mat4);
  void setUniform(const char* name, const Eigen::Vector4f& vec4);

 private:
  static GLuint currentBinding;
  bool isLinked;
  GLuint handle;
  mutable std::unordered_map<std::string, GLint> uniforms;
  mutable std::unordered_map<std::string, GLint> uniformBlocks;
};
