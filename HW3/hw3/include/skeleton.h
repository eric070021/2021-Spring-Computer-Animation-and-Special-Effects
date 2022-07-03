#pragma once
#include <string>
#include <vector>

#include "bone.h"

class Skeleton {
 public:
  explicit Skeleton(const std::string &filename, float scale_ = 0.2f) noexcept;
  /**
   * @brief Get skeleton's scale
   */
  float scale() const { return _scale; }
  /**
   * @brief Get total bones in the skeleton
   */
  int size() const { return static_cast<int>(bones.size()); }
  /**
   * @brief Get total movable bones in the skeleton
   */
  int movableBoneNum() const { return _movableBones; }
  /**
   * @brief Get specific bone by its index
   */
  Bone *bone(int bone_idx) { return &bones[bone_idx]; }
  /**
   * @brief Get specific bone by its index
   */
  const Bone *const bone(int bone_idx) const { return &bones[bone_idx]; }
  /**
   * @brief Get specific bone by its name
   */
  Bone *bone(const std::string &name);
  /**
   * @brief Get specific bone by its index
   */
  const Bone *const bone(const std::string &name) const;
  /**
   * @brief Set bone's model matrices (for rendering)
   */
  void setModelMatrix(Eigen::Ref<Eigen::Matrix4Xf> modelMatrix);

 private:
  /**
   * @brief Read Acclaim Skeleton File
   */
  bool readASFFile(const std::string &file_name);
  /**
   * @brief This function sets sibling or child for parent bone.
   * If parent bone does not have a child, then `child` is set as parent's child
   * else `child` is set as a sibling of parents already existing child
   */
  int setChildrenSibling(Bone *parent, Bone *child);
  /**
   * @brief Transform the direction vector,
   * which is defined in character's global coordinate system in the ASF file, to local coordinate
   */
  void rotateLocalCoordinate();
  /**
   * @brief Rotation from parent bone to child bone.
   */
  void computeRotationParent2Child();

  float _scale = 0.2f;
  int _movableBones = 1;
  std::vector<Bone> bones = std::vector<Bone>(1);
};
