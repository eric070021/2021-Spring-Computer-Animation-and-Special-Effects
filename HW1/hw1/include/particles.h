#pragma once
#include <Eigen/Core>
#include <vector>

class Particles {
 public:
  Particles(int size = -1, float mass_ = 0.0f) noexcept;
  void resize(int newSize);
  void setZero();

  int getCapacity() const { return static_cast<int>(_position.cols()); }
  // Get all particles.
  Eigen::Ref<Eigen::Matrix4Xf> position() { return _position; }
  Eigen::Ref<Eigen::Matrix4Xf> velocity() { return _velocity; }
  Eigen::Ref<Eigen::Matrix4Xf> acceleration() { return _acceleration; }
  std::vector<float>& mass() { return _mass; }
  // Get specific particle by index.
  Eigen::Ref<Eigen::Vector4f> position(int i) { return _position.col(i); }
  Eigen::Ref<Eigen::Vector4f> velocity(int i) { return _velocity.col(i); }
  Eigen::Ref<Eigen::Vector4f> acceleration(int i) { return _acceleration.col(i); }
  float& mass(int i) { return _mass[i]; }
  float inverseMass(int i) { return (_mass[i] == 0.0f) ? 0.0f : 1.0f / _mass[i]; }

  const float* getPositionData() const { return _position.data(); }
  const float* getVelocityData() const { return _velocity.data(); }
  const float* getAccelerationData() const { return _acceleration.data(); }
  const float* getMassData() const { return _mass.data(); }

 private:
  Eigen::Matrix4Xf _position;
  Eigen::Matrix4Xf _velocity;
  Eigen::Matrix4Xf _acceleration;
  std::vector<float> _mass;
};
