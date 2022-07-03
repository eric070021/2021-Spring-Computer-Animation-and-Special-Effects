#pragma once
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Bone segment names used in ASF file
// this structure defines the property of each bone segment, including its
// connection to other bones, DOF (degrees of freedom), relative orientation and
// distance to the outboard bone
struct Bone {
  // Pointer to the sibling (branch bone) in the hierarchy tree
  Bone *sibling = nullptr;
  // Pointer to the child (outboard bone) in the hierarchy tree
  Bone *child = nullptr;
  // Pointer to the parent (inboard bone) in the hierarchy tree
  Bone *parent = nullptr;
  // Bone index
  int idx = 0;
  // Unit vector describes the direction from local origin to
  // the origin of the child bone
  // Notice: stored in local coordinate system of the bone
  Eigen::Vector3f direction = Eigen::Vector3f::Zero();
  // Bone name
  std::string name = "";
  // Bone length
  float length = 0.0;
  // Orientation of each bone's local coordinate system as specified in ASF file (axis field)
  // Stored as R = Rz * Ry * Rx
  Eigen::Quaternionf axis = Eigen::Quaternionf::Identity();
  // number of bone's degrees of freedom
  int dof = 0;
  // degree of freedom mask in x, y, z axis (local)
  bool dofrx = false, dofry = false, dofrz = false;  // Rotate
  bool doftx = false, dofty = false, doftz = false;  // Translate
  // Rotation matrix from parent to child
  Eigen::Quaternionf rotationParentCurrent = Eigen::Quaternionf::Identity();
  // Initial rotation and scaling for bone
  Eigen::Affine3f globalFacing = Eigen::Affine3f::Identity();
  // Bone's start pos in global position
  Eigen::Vector3f startPosition = Eigen::Vector3f::Zero();
  // Bone's end pos in global position
  Eigen::Vector3f endPosition = Eigen::Vector3f::Zero();
  // Bone's rotation in global position
  Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
  // Bone's rotation limits in radian, which in [-pi, pi]
  float rxmin = 0.0f, rxmax = 0.0f;
  float rymin = 0.0f, rymax = 0.0f;
  float rzmin = 0.0f, rzmax = 0.0f;
};
