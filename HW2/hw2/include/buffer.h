#pragma once
#include <vector>

#include <glad/gl.h>

#include "utils.h"

class Buffer {
 public:
  MOVE_ONLY(Buffer)
  /**
   * @brief Construct a new Buffer object. Allocate a handle.
   *
   */
  Buffer() noexcept;
  /**
   * @brief Destroy the Buffer object. Release the handle.
   *
   */
  virtual ~Buffer();
  /**
   * @brief Bind the buffer.
   *
   */
  void bind() const noexcept;
  /**
   * @brief Allocate memory for the buffer, call on a exist object will reallocate it.
   *
   * @param _size Buffer size in bytes.
   * @param usage One of the buffer usage macro. GL_[STATIC/DYNAMIC/STREAM]_[READ/COPY/DRAW]
   */
  void allocate(GLsizeiptr _size, GLenum usage = GL_DYNAMIC_DRAW) const noexcept;
  /**
   * @brief Load data to the buffer. The buffer must be allocated.
   *
   * @param offset The offset into the buffer object's data store in bytes.
   * @param _size The size of the data in bytes.
   * @param data Pointer to the data to be loaded.
   */
  void load(GLintptr offset, GLsizeiptr _size, const void* data) const noexcept;
  /**
   * @brief Do allocation and load in once, useful for constant data.
   *
   * @param _size The size of the data in bytes.
   * @param data Pointer to the data to be loaded.
   * @param usage One of the buffer usage macro. GL_[STATIC/DYNAMIC/STREAM]_[READ/COPY/DRAW]
   */
  void allocate_load(GLsizeiptr _size, const void* data, GLenum usage = GL_STATIC_DRAW) const noexcept;
  /**
   * @brief Get the type string
   *
   * @return String description of the buffer type.
   */
  CONSTEXPR_VIRTUAL virtual const char* getTypeName() const noexcept = 0;
  /**
   * @brief Get the type enum
   *
   * @return OpenGL enum of the buffer type.
   */
  CONSTEXPR_VIRTUAL virtual GLenum getType() const noexcept = 0;
  /**
   * @brief Get the internal handle of the object
   *
   * @return GLuint handle of the object.
   */
  GLuint handle() const noexcept { return _handle; }
  /**
   * @brief Get the allocated size of the buffer.
   *
   * @return Size of the allocated buffer in bytes.
   */
  GLsizeiptr size() const noexcept { return _size; }

 protected:
  GLuint _handle;
  mutable GLsizeiptr _size;
};

class ArrayBuffer final : public Buffer {
 public:
  /**
   * @brief Get the type string
   *
   * @return String description of the buffer type.
   */
  CONSTEXPR_VIRTUAL const char* getTypeName() const noexcept override { return "Array buffer"; }
  /**
   * @brief Get the type enum
   *
   * @return OpenGL enum of the buffer type.
   */
  CONSTEXPR_VIRTUAL GLenum getType() const noexcept override { return GL_ARRAY_BUFFER; }
};

class ElementArrayBuffer final : public Buffer {
 public:
  /**
   * @brief Get the type string
   *
   * @return String description of the buffer type.
   */
  CONSTEXPR_VIRTUAL const char* getTypeName() const noexcept override { return "Element array buffer"; }
  /**
   * @brief Get the type enum
   *
   * @return OpenGL enum of the buffer type.
   */
  CONSTEXPR_VIRTUAL GLenum getType() const noexcept override { return GL_ELEMENT_ARRAY_BUFFER; }
};

class UniformBuffer final : public Buffer {
 public:
  /**
   * @brief Get the type string
   *
   * @return String description of the buffer type.
   */
  CONSTEXPR_VIRTUAL const char* getTypeName() const noexcept override { return "Uniform buffer"; }
  /**
   * @brief Get the type enum
   *
   * @return OpenGL enum of the buffer type.
   */
  CONSTEXPR_VIRTUAL GLenum getType() const noexcept override { return GL_UNIFORM_BUFFER; }
  /**
   * @brief  Bind a range of buffer to a given uniform buffer index.
   *
   * @param index Uniform buffer index.
   * @param offset The starting offset of the buffer in bytes.
   * @param _size The size of binded range in bytes.
   */
  void bindUniformBlockIndex(GLuint index, GLuint offset, GLuint _size) const noexcept;
  /**
   * @brief Bind whole buffer to a given uniform buffer index.
   *
   * @param index Uniform buffer index.
   */
  void bindUniformBlockIndex(GLuint index) const noexcept;
};
