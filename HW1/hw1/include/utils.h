#pragma once
#include <filesystem>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#ifndef DELETE_COPY
#define DELETE_COPY(ClassName)           \
  ClassName(const ClassName &) = delete; \
  ClassName &operator=(const ClassName &) = delete;
#endif

#ifndef DEFAULT_COPY
#define DEFAULT_COPY(ClassName)           \
  ClassName(const ClassName &) = default; \
  ClassName &operator=(const ClassName &) = default;
#endif

#ifndef DELETE_MOVE
#define DELETE_MOVE(ClassName)      \
  ClassName(ClassName &&) = delete; \
  ClassName &operator=(ClassName &&) = delete;
#endif

#ifndef DEFAULT_MOVE
#define DEFAULT_MOVE(ClassName)      \
  ClassName(ClassName &&) = default; \
  ClassName &operator=(ClassName &&) = default;
#endif

#ifndef MOVE_ONLY
#define MOVE_ONLY(ClassName) \
  DELETE_COPY(ClassName)     \
  DEFAULT_MOVE(ClassName)
#endif

// Some useful C++ 20 feature
#if __cplusplus >= 202002L && !defined(__APPLE__)
#ifndef CONSTEXPR_VIRTUAL
#define CONSTEXPR_VIRTUAL constexpr
#endif
#endif

// Fallbacks
#ifndef CONSTEXPR_VIRTUAL
#define CONSTEXPR_VIRTUAL
#endif

constexpr float toRadians(double x) { return static_cast<float>(x * EIGEN_PI / 180); }

Eigen::Matrix4f lookAt(const Eigen::Ref<const Eigen::Vector4f> &position,
                       const Eigen::Ref<const Eigen::Vector4f> &front,
                       const Eigen::Ref<const Eigen::Vector4f> &up);
Eigen::Matrix4f perspective(float fov, float aspect, float zNear, float zFar);
Eigen::Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar);
std::filesystem::path findPath(const std::string &filename);
