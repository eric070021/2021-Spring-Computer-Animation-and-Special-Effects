#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

struct Posture final {
  Posture() noexcept = default;
  explicit Posture(const std::size_t size) noexcept;

  std::vector<Eigen::Vector3f> eulerAngle;
  std::vector<Eigen::Quaternionf> rotations;
  std::vector<Eigen::Vector3f> translations;
};
