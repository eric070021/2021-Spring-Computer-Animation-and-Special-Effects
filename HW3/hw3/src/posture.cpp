#include "posture.h"

#include <utility>

Posture::Posture(const std::size_t size) noexcept :
    eulerAngle(size, Eigen::Vector3f::Zero()),
    rotations(size, Eigen::Quaternionf::Identity()),
    translations(size, Eigen::Vector3f::Zero()) {}
