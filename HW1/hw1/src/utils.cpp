#include "utils.h"
#include <Eigen/Dense>
#include <iostream>

using Eigen::Matrix4f;
using Eigen::Vector4f;

namespace {
std::filesystem::path findAssetPath() {
  namespace fs = std::filesystem;
  std::filesystem::path assetPath = fs::current_path();
  while (!fs::exists(assetPath / "assets/.placeholder") && assetPath.has_relative_path()) {
    assetPath = assetPath.parent_path();
  }
  if (!fs::exists(assetPath / "assets/.placeholder")) {
    std::cerr << "Asset not found" << std::endl;
    exit(EXIT_FAILURE);
  }
  assetPath /= "assets";
  return assetPath;
}
}  // namespace

Matrix4f lookAt(const Eigen::Ref<const Eigen::Vector4f>& position,
                const Eigen::Ref<const Eigen::Vector4f>& front,
                const Eigen::Ref<const Eigen::Vector4f>& up) {
  Vector4f f = front.normalized();
  Vector4f u = up.normalized();
  Vector4f s = f.cross3(u).normalized();
  u = s.cross3(f);

  Matrix4f mat(Matrix4f::Zero());
  mat.row(0) = s;
  mat.row(1) = u;
  mat.row(2) = -f;
  mat.row(3) << 0, 0, 0, 1;
  mat(0, 3) = -s.dot(position);
  mat(1, 3) = -u.dot(position);
  mat(2, 3) = f.dot(position);
  return mat;
}

Matrix4f perspective(float fov, float aspect, float zNear, float zFar) {
  Eigen::Transform<float, 3, Eigen::Projective> tr;
  tr.matrix().setZero();

  float tanFOV2 = std::tan(fov / 2.0f);
  tr(0, 0) = 1.0f / (aspect * tanFOV2);
  tr(1, 1) = 1.0f / (tanFOV2);
  tr(2, 2) = -(zFar + zNear) / (zFar - zNear);
  tr(3, 2) = -1.0f;
  tr(2, 3) = -(2.0f * zFar * zNear) / (zFar - zNear);
  return tr.matrix();
}

Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar) {
  Matrix4f mat = Matrix4f::Identity();
  mat(0, 0) = 2.0f / (right - left);
  mat(1, 1) = 2.0f / (top - bottom);
  mat(2, 2) = -2.0f / (zFar - zNear);
  mat(3, 0) = -(right + left) / (right - left);
  mat(3, 1) = -(top + bottom) / (top - bottom);
  mat(3, 2) = -(zFar + zNear) / (zFar - zNear);
  return mat;
}

std::filesystem::path findPath(const std::string& filename) {
  static std::filesystem::path assetPath = findAssetPath();
  return assetPath / filename;
}
