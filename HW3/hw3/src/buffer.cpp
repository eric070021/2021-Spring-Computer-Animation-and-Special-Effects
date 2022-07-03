#include "buffer.h"

Buffer::Buffer() noexcept : _handle(0), _size(0) { glGenBuffers(1, &_handle); }

Buffer::~Buffer() { glDeleteBuffers(1, &_handle); }

void Buffer::bind() const noexcept { glBindBuffer(getType(), _handle); }

void Buffer::allocate(GLsizeiptr size_, GLenum usage) const noexcept {
  bind();
  _size = size_;
  glBufferData(getType(), _size, nullptr, usage);
}

void Buffer::load(GLintptr offset, GLsizeiptr size_, const void* data) const noexcept {
  bind();
  glBufferSubData(getType(), offset, size_, data);
}

void Buffer::allocate_load(GLsizeiptr size_, const void* data, GLenum usage) const noexcept {
  bind();
  _size = size_;
  glBufferData(getType(), _size, data, usage);
}

void UniformBuffer::bindUniformBlockIndex(GLuint index, GLuint offset, GLuint size_) const noexcept {
  bind();
  glBindBufferRange(GL_UNIFORM_BUFFER, index, _handle, offset, size_);
  glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void UniformBuffer::bindUniformBlockIndex(GLuint index) const noexcept {
  bind();
  glBindBufferBase(GL_UNIFORM_BUFFER, index, _handle);
  glBindBuffer(GL_UNIFORM_BUFFER, 0);
}
