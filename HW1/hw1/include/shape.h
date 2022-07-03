#pragma once
#include <Eigen/Core>
#include <functional>
#include "particles.h"

class Cloth;
class Spheres;

class Integrator;
class Shape {
 public:
  explicit Shape(int size, float mass_) noexcept;
  virtual ~Shape() = default;

  void setModelMatrix(const Eigen::Ref<const Eigen::Matrix4f>& _modelMatrix);
  Particles& particles() { return _particles; }
  Eigen::Matrix4f getModelMatrix() const { return modelMatrix; }
  Eigen::Matrix4f getNormalMatrix() const { return normalMatrix; }
  /**
   * @brief Compute gravity and viscous force.
   *
   */
  void computeExternalForce();
  virtual void collide(Shape* shape) = 0;
  virtual void collide(Cloth*) { return; }
  virtual void collide(Spheres*) { return; }
  virtual void collide() { return; }

 protected:
  Particles _particles;
  Eigen::Matrix4f modelMatrix;
  Eigen::Matrix4f normalMatrix;
};
