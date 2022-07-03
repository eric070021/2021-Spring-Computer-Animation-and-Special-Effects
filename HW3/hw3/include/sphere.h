#pragma once
#include <Eigen/Core>
#include <vector>

#include "buffer.h"
#include "utils.h"
#include "vertexarray.h"

class Sphere final {
 public:
  MOVE_ONLY(Sphere)
  explicit Sphere(int size) noexcept;
  void draw();

  Eigen::Ref<Eigen::Matrix4f> modelMatrix(int index) {
    isUpdated = true;
    return _modelMatrix.block<4, 4>(0, index * 4);
  }

  Eigen::Ref<Eigen::Matrix4Xf> modelMatrix() {
    isUpdated = true;
    return _modelMatrix;
  }

 private:
  Eigen::Matrix4Xf _modelMatrix;
  bool isUpdated;
  VertexArray vao;
  ArrayBuffer vbo;
  ArrayBuffer models;
  ElementArrayBuffer ebo;
};
