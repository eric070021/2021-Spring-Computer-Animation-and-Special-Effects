#pragma once
#include <Eigen/Core>
#include <functional>
#include <vector>

#include "particles.h"
#include "utils.h"

class Integrator {
 public:
  Integrator() noexcept {}
  DELETE_COPY(Integrator)
  DELETE_MOVE(Integrator)
  enum class Type { EXPLICIT_EULER, IMPLICIT_EULER, MIDPOINT_EULER, RUNGE_KUTTA_FOURTH };
  /**
   * @brief Integrate the ODE of acceleration and velocity.
   *
   * @param particles A vector of particles to be integrated.
   * @param simulateOneStep A function that computes next step f(x, t+h)
   */
  virtual void integrate(const std::vector<Particles *> &particles,
                         std::function<void(void)> simulateOneStep) const = 0;
  CONSTEXPR_VIRTUAL virtual Type getType() const = 0;
};

class ExplicitEuler : public Integrator {
 public:
  void integrate(const std::vector<Particles *> &particles, std::function<void(void)> simulateOneStep) const override;
  CONSTEXPR_VIRTUAL Type getType() const override { return Type::EXPLICIT_EULER; }
};

class ImplicitEuler : public Integrator {
 public:
  void integrate(const std::vector<Particles *> &particles, std::function<void(void)> simulateOneStep) const override;
  CONSTEXPR_VIRTUAL Type getType() const override { return Type::IMPLICIT_EULER; }
};

class MidpointEuler : public Integrator {
 public:
  void integrate(const std::vector<Particles *> &particles, std::function<void(void)> simulateOneStep) const override;
  CONSTEXPR_VIRTUAL Type getType() const override { return Type::MIDPOINT_EULER; }
};

class RungeKuttaFourth : public Integrator {
 public:
  void integrate(const std::vector<Particles *> &particles, std::function<void(void)> simulateOneStep) const override;
  CONSTEXPR_VIRTUAL Type getType() const override { return Type::RUNGE_KUTTA_FOURTH; }
};
