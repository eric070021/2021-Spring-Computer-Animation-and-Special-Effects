#pragma once
#include <Eigen/Core>
#include <vector>

#include "buffer.h"
#include "shape.h"
#include "utils.h"
#include "vertexarray.h"

class Spheres final : public Shape {
 public:
  MOVE_ONLY(Spheres)
  static Spheres& initSpheres();
  void addSphere(const Eigen::Ref<const Eigen::Vector4f>& position, float size);
  void draw() const;
  void collide(Shape* shape) override;
  void collide(Cloth* cloth) override;
  void collide() override;
  float radius(int i) const { return _radius[i]; }

 private:
  Spheres();

  int sphereCount;
  std::vector<float> _radius;
  VertexArray vao;
  ArrayBuffer vbo;
  ArrayBuffer offsets;
  ArrayBuffer sizes;
  ElementArrayBuffer ebo;
};
