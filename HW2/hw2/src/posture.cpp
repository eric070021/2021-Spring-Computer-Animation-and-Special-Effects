#include "posture.h"

#include <utility>

Posture::Posture(const std::size_t size) noexcept :
    rotations(size, Eigen::Quaternionf::Identity()), translations(size, Eigen::Vector3f::Zero()) {}
