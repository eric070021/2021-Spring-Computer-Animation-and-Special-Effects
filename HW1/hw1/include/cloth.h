#pragma once
#include <glad/gl.h>
#include <vector>

#include "buffer.h"
#include "shape.h"
#include "spring.h"
#include "utils.h"
#include "vertexarray.h"

class Cloth final : public Shape {
 public:
  MOVE_ONLY(Cloth)
  enum class DrawType { FULL, STRUCTURAL, SHEAR, BEND, PARTICLE };
  Cloth();
  /**
   * @brief Get the springs.
   *
   */
  std::vector<Spring>& springs() { return _springs; }
  /**
   * @brief Render the cloth based on the given type.
   *
   * @param type The render type.
   */
  void draw(DrawType type) const;
  /**
   * @brief Compute the internal force produce by the springs.
   * Which includes spring force and damper force.
   *
   */
  void computeSpringForce();
  /**
   * @brief Compute the smooth normal of the surface. Only called when draw type is FULL
   *
   */
  void computeNormal();
  /**
   * @brief Cloth collide with unknown shape
   *
   * @param shape The shape to be tested.
   */
  void collide(Shape* shape) override;
  /**
   * @brief Cloth collide with sphere.
   *
   * @param sphere The sphere to be tested.
   */
  void collide(Spheres* sphere) override;

 private:
  /**
   * @brief Initialize the model, setting OpenGL related buffers.
   *
   */
  void initializeVertex();
  /**
   * @brief Connect particles with springs.
   *
   */
  void initializeSpring();
  std::vector<Spring> _springs;
  VertexArray vao;
  ArrayBuffer positionBuffer;
  ArrayBuffer normalBuffer;
  ElementArrayBuffer ebo, structuralSpring, shearSpring, bendSpring;
};
