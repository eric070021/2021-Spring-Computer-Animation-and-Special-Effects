#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

struct GLFWwindow;
class Camera {
 public:
  /**
   * @brief Construct a new Camera.
   *
   * @param position The position of the camera.
   */
  explicit Camera(const Eigen::Ref<const Eigen::Vector3f>& position,
                  Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity());
  /**
   * @brief Get the front vector of the camera. (Camera facing)
   */
  Eigen::Ref<Eigen::Vector3f> front() { return _front; }
  /**
   * @brief Get the position of the camera.
   */
  Eigen::Ref<Eigen::Vector3f> position() { return _position; }
  /**
   * @brief Get the up vector of the camera.
   */
  Eigen::Ref<Eigen::Vector3f> up() { return _up; }
  /**
   * @brief Get the right vector of the camera.
   */
  Eigen::Ref<Eigen::Vector3f> right() { return _right; }
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
  Eigen::Vector3f _front;
  Eigen::Vector3f _position;
  Eigen::Vector3f _up;
  Eigen::Vector3f _right;
  Eigen::Matrix4f _projectionMatrix;
  Eigen::Matrix4f _viewMatrix;
  Eigen::Quaternionf _rotation;
};
