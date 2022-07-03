#include "integrator.h"

#include "configs.h"

struct tempState {
  Eigen::Matrix4Xf acceleration;
  Eigen::Matrix4Xf velocity;
  Eigen::Matrix4Xf position;
};

void ExplicitEuler::integrate(const std::vector<Particles *> &particles, std::function<void(void)>) const {
  // TODO: Integrate velocity and acceleration
  //   1. Integrate velocity.
  //   2. Integrate acceleration.
  //   3. You should not compute position using acceleration. Since some part only update velocity. (e.g. impulse)
  // Note:
  //   1. You don't need the simulation function in explicit euler.
  //   2. You should do this first because it is very simple. Then you can chech your collision is correct or not.
  //   3. This can be done in 2 lines. (Hint: You can add / multiply all particles at once since it is a large matrix.)
  for (const auto &p : particles) {
    p->velocity() += deltaTime * p->acceleration();
    p->position() += deltaTime * p->velocity(); 
  }
}

void ImplicitEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  std::vector<tempState> origin;
  for (const auto &p : particles) {
    tempState o;
    o.velocity = p->velocity();
    o.position = p->position();
    Eigen::Matrix4Xf euler_velocity = deltaTime * p->acceleration();
    Eigen::Matrix4Xf euler_position = deltaTime * p->velocity();
    p->velocity() += euler_velocity ;
    p->position() += euler_position ;
    origin.push_back(o);
  }
  simulateOneStep();

  int i = 0;
  for (const auto &p : particles) {
    p->velocity() = origin[i].velocity + deltaTime * p->acceleration();
    p->position() = origin[i].position + deltaTime * p->velocity();
    i++;
  }
}

void MidpointEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  std::vector<tempState> v;
  for (const auto &p : particles) {
    tempState t;
    t.velocity = p->velocity();
    t.position = p->position();
    v.push_back(t);
    Eigen::Matrix4Xf euler_velocity = deltaTime * p->acceleration();
    Eigen::Matrix4Xf euler_position = deltaTime * p->velocity();
    p->velocity() += euler_velocity / 2;
    p->position() += euler_position / 2;
  }
  simulateOneStep();

  int i = 0;
  for (const auto &p : particles) {
    p->velocity() = v[i].velocity + deltaTime * p->acceleration();
    p->position() = v[i].position + deltaTime * v[i].velocity; 
    i++;
  }
}

void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Compute k1, k2, k3, k4
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

    /*std::vector<tempState> origin, vk1, vk2, vk3, vk4;
    for (const auto &p : particles) {
    tempState o, k1, k2;
    o.velocity = p->velocity();
    o.position = p->position();
    k1.acceleration = deltaTime * p->acceleration();
    k1.velocity = deltaTime * p->velocity();
    p->velocity() = p->velocity() + deltaTime * p->acceleration() / 2;
    p->position() = p->position() + deltaTime * p->velocity() / 2;
    k2.velocity = deltaTime * p->velocity();

    origin.push_back(o);
    vk1.push_back(k1);
    vk2.push_back(k2);
    }
    simulateOneStep();
    int i =  0;
    for (const auto &p : particles) {
    tempState k3;
    vk2[i].acceleration = deltaTime * p->acceleration();
    p->velocity() = origin[i].velocity + vk2[i].acceleration / 2;
    p->position() = origin[i].position + vk2[i].velocity / 2;
    i++;
    k3.velocity = deltaTime * p->velocity();
    vk3.push_back(k3);
    }
    simulateOneStep();
    i = 0;
    for (const auto &p : particles) {
    tempState k4;
    vk3[i].acceleration = deltaTime * p->acceleration();
    p->velocity() = origin[i].velocity + vk3[i].acceleration;
    p->position() = origin[i].position + vk3[i].velocity;
    i++;
    k4.velocity = deltaTime * p->velocity();
    vk4.push_back(k4);
    }
    simulateOneStep();

    i = 0;
    for (const auto &p : particles) {
      vk4[i].acceleration = deltaTime * p->acceleration();

      p->velocity() = origin[i].velocity +
          (vk1[i].acceleration + 2 * vk2[i].acceleration + 2 * vk3[i].acceleration + vk4[i].acceleration) / 6;
      p->position() =
          origin[i].position + (vk1[i].velocity + 2 * vk2[i].velocity + 2 * vk3[i].velocity + vk4[i].velocity) / 6;
      i++;
    }*/

    std::vector<tempState> origin, vk1, vk2, vk3, vk4;
    for (const auto &p : particles) {
      tempState o, k1;
      o.velocity = p->velocity();
      o.position = p->position();
      k1.acceleration = deltaTime * p->acceleration();
      k1.velocity = deltaTime * p->velocity();
      p->velocity() = p->velocity() + deltaTime * p->acceleration() / 2;
      p->position() = p->position() + deltaTime * p->velocity() / 2;
 
      origin.push_back(o);
      vk1.push_back(k1);
    }
    simulateOneStep();
    int i = 0;
    for (const auto &p : particles) {
      tempState k2;
      k2.acceleration = deltaTime * p->acceleration();
      k2.velocity = deltaTime * p->velocity();
      p->velocity() = origin[i].velocity + deltaTime * p->acceleration() / 2;
      p->position() = origin[i].position + deltaTime * p->velocity() / 2;

      i++;
      vk2.push_back(k2);
    }
    simulateOneStep();
    i = 0;
    for (const auto &p : particles) {
      tempState k3;
      k3.acceleration = deltaTime * p->acceleration();
      k3.velocity = deltaTime * p->velocity();
      p->velocity() = origin[i].velocity + deltaTime * p->acceleration();
      p->position() = origin[i].position + deltaTime * p->velocity();
      i++;
      vk3.push_back(k3);
    }
    simulateOneStep();

    i = 0;
    for (const auto &p : particles) {
      tempState k4;
      k4.acceleration = deltaTime * p->acceleration();
      k4.velocity = deltaTime * p->velocity();

      p->velocity() =
          origin[i].velocity +
                      (vk1[i].acceleration + 2 * vk2[i].acceleration + 2 * vk3[i].acceleration + k4.acceleration) / 6;
      p->position() =
          origin[i].position + (vk1[i].velocity + 2 * vk2[i].velocity + 2 * vk3[i].velocity + k4.velocity) / 6;
      i++;
      vk4.push_back(k4);
    }
}
