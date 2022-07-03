#include "cylinder.h"

#include "configs.h"

namespace {
void generateVertices(std::vector<GLfloat>& vertices, std::vector<GLuint>& indices) {
  // http://www.songho.ca/opengl/gl_cylinder.html#cylinderconst
  vertices.reserve(2 * 6 * (2 * cylinderSectors + 1) + 2);

  float sectorStep = static_cast<float>(2.0 * EIGEN_PI / cylinderSectors);
  float sectorAngle = 0;  // radian
  std::vector<float> unitCircle((cylinderSectors + 1) * 3);

  int currentPos = -1;
  for (int i = 0; i <= cylinderSectors; ++i) {
    unitCircle[++currentPos] = cosf(sectorAngle);  // x
    unitCircle[++currentPos] = sinf(sectorAngle);  // y
    unitCircle[++currentPos] = 0;                  // z
    sectorAngle += sectorStep;
  }
  // put side vertices to arrays
  for (int i = 0; i < 2; ++i) {
    float h = -cylinderHeight / 2.0f + i * cylinderHeight;  // z value; -h/2 to h/2

    for (int j = 0, k = 0; j <= cylinderSectors; ++j, k += 3) {
      float ux = unitCircle[k];
      float uy = unitCircle[k + 1];
      float uz = unitCircle[k + 2];
      vertices.insert(vertices.end(), {ux * cylinderRadius, uy * cylinderRadius, h, ux, uy, uz});
    }
  }
  // the starting index for the base/top surface
  // NOTE: it is used for generating indices later
  int baseCenterIndex = static_cast<int>(vertices.size()) / 6;
  int topCenterIndex = baseCenterIndex + cylinderSectors + 1;  // include center vertex

  // put base and top vertices to arrays
  for (int i = 0; i < 2; ++i) {
    float h = -cylinderHeight / 2.0f + i * cylinderHeight;  // z value; -h/2 to h/2
    float nz = -1.0f + 2.0f * i;                            // z value of normal; -1 to 1

    // center point
    vertices.insert(vertices.end(), {0, 0, h, 0, 0, nz});

    for (int j = 0, k = 0; j < cylinderSectors; ++j, k += 3) {
      float ux = unitCircle[k];
      float uy = unitCircle[k + 1];
      vertices.insert(vertices.end(), {ux * cylinderRadius, uy * cylinderRadius, h, 0, 0, nz});
    }
  }

  int k1 = 0;                    // 1st vertex index at base
  int k2 = cylinderSectors + 1;  // 1st vertex index at top

  // indices for the side surface
  for (int i = 0; i < cylinderSectors; ++i, ++k1, ++k2) {
    // 2 triangles per sector
    // k1 => k1+1 => k2
    indices.push_back(k1);
    indices.push_back(k1 + 1);
    indices.push_back(k2);

    // k2 => k1+1 => k2+1
    indices.push_back(k2);
    indices.push_back(k1 + 1);
    indices.push_back(k2 + 1);
  }

  // indices for the base surface
  // NOTE: baseCenterIndex and topCenterIndices are pre-computed during vertex generation
  //      please see the previous code snippet
  for (int i = 0, k = baseCenterIndex + 1; i < cylinderSectors; ++i, ++k) {
    if (i < cylinderSectors - 1) {
      indices.push_back(baseCenterIndex);
      indices.push_back(k + 1);
      indices.push_back(k);
    } else {
      indices.push_back(baseCenterIndex);
      indices.push_back(baseCenterIndex + 1);
      indices.push_back(k);
    }
  }

  // indices for the top surface
  for (int i = 0, k = topCenterIndex + 1; i < cylinderSectors; ++i, ++k) {
    if (i < cylinderSectors - 1) {
      indices.push_back(topCenterIndex);
      indices.push_back(k);
      indices.push_back(k + 1);
    } else {
      indices.push_back(topCenterIndex);
      indices.push_back(k);
      indices.push_back(topCenterIndex + 1);
    }
  }
}
}  // namespace

Cylinder::Cylinder(int size) noexcept : _modelMatrix(4, size * 4), isUpdated(false) {
  models.allocate(16 * size * sizeof(GLfloat));
  for (int i = 0; i < size; i++) modelMatrix(i).setIdentity();
  models.load(0, 16 * size * sizeof(GLfloat), _modelMatrix.data());

  std::vector<GLfloat> vertices;
  std::vector<GLuint> indices;
  generateVertices(vertices, indices);

  vbo.allocate_load(vertices.size() * sizeof(GLfloat), vertices.data());
  ebo.allocate_load(indices.size() * sizeof(GLuint), indices.data());

  vao.bind();
  vbo.bind();
  ebo.bind();

  vao.enable(0);
  vao.setAttributePointer(0, 3, 6, 0);
  glVertexAttribDivisor(0, 0);
  vao.enable(1);
  vao.setAttributePointer(1, 3, 6, 3);
  glVertexAttribDivisor(1, 0);
  models.bind();
  vao.enable(2);
  vao.setAttributePointer(2, 4, 16, 0);
  glVertexAttribDivisor(2, 1);
  vao.enable(3);
  vao.setAttributePointer(3, 4, 16, 4);
  glVertexAttribDivisor(3, 1);
  vao.enable(4);
  vao.setAttributePointer(4, 4, 16, 8);
  glVertexAttribDivisor(4, 1);
  vao.enable(5);
  vao.setAttributePointer(5, 4, 16, 12);
  glVertexAttribDivisor(5, 1);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Cylinder::draw() {
  int nCylinder = static_cast<int>(_modelMatrix.cols()) / 4;
  if (isUpdated) {
    models.load(0, 16 * nCylinder * sizeof(GLfloat), _modelMatrix.data());
    isUpdated = false;
  }
  vao.bind();
  GLsizei indexCount = static_cast<GLsizei>(ebo.size() / sizeof(GLuint));
  glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr, nCylinder);
}
