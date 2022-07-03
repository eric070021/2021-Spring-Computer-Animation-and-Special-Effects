#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

struct GLFWwindow;
class Camera {
 public:
  /**
   * @brief Construct a new Camera.
   *
   * @param position The position of the camera. This w value should be 1.0f
   */
  explicit Camera(const Eigen::Ref<const Eigen::Vector4f>& position);
  /**
   * @brief Get the front vector of the camera. (Camera facing)
   */
  Eigen::Ref<Eigen::Vector4f> front() { return _front; }
  /**
   * @brief Get the position of the camera.
   */
  Eigen::Ref<Eigen::Vector4f> position() { return _position; }
  /**
   * @brief Get the up vector of the camera.
   */
  Eigen::Ref<Eigen::Vector4f> up() { return _up; }
  /**
   * @brief Get the right vector of the camera.
   */
  Eigen::Ref<Eigen::Vector4f> right() { return _right; }
  /**
   * @brief Get the 4*4 projection matrix of the camera.
   */
  Eigen::Ref<Eigen::Matrix4f> projectionMatrix() { return _projectionMatrix; }
  /**
   * @brief Get the 4*4 view matrix of the camera.
   */
  Eigen::Ref<Eigen::Matrix4f> viewMatrix() { return _viewMatrix; }
  /**
   * @brief Projection * view.
   */
  Eigen::Matrix4f viewProjectionMatrix() { return _projectionMatrix * _viewMatrix; }
  /**
   * @brief Process keyboard and mouse event to move camera
   *
   * @param window The current active window.
   * @return true if the event move the camera.
   */
  bool move(GLFWwindow* window);
  /**
   * @brief Recalculate the projection matrix. Need to be call when window size changes.
   *
   */
  void updateProjection();
  /**
   * @brief Recalculate the view matrix.
   *
   */
  void updateView();

 private:
  Eigen::Vector4f _front;
  Eigen::Vector4f _position;
  Eigen::Vector4f _up;
  Eigen::Vector4f _right;
  Eigen::Matrix4f _projectionMatrix;
  Eigen::Matrix4f _viewMatrix;
  Eigen::Quaternionf _rotation;
};
