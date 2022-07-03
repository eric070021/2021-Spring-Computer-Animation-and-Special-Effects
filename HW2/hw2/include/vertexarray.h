#pragma once
#include <glad/gl.h>

#include "utils.h"

class VertexArray {
 public:
  MOVE_ONLY(VertexArray)
  VertexArray();
  ~VertexArray();

  void enable(int index) const;
  void setAttributePointer(int index, int size, int stride, int offset) const;
  void bind() const;

 private:
  GLuint handle;
};
