#include "skeleton.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include "utils.h"

Skeleton::Skeleton(const std::string &file_name, float scale_) noexcept : _scale(scale_) {
  bones.reserve(64);
  // Initializaton of root bone
  bones[0].name = std::string("root");
  bones[0].idx = 0;
  bones[0].parent = nullptr;
  bones[0].sibling = nullptr;
  bones[0].child = nullptr;
  bones[0].direction.setZero();
  bones[0].length = 0;
  bones[0].dof = 6;
  bones[0].dofrx = true;
  bones[0].dofry = true;
  bones[0].dofrz = true;
  bones[0].doftx = true;
  bones[0].dofty = true;
  bones[0].doftz = true;
  // build hierarchy and read in each bone's DOF information
  readASFFile(file_name);
  // transform the direction vector for each bone from the world coordinate system
  // to it's local coordinate system
  rotateLocalCoordinate();
  // Calculate rotation from each bone local coordinate system to the coordinate system of its parent
  // store it in rotationParentCurrent variable for each bone
  computeRotationParent2Child();
  // Compute global facing of the bones
  for (size_t i = 0; i < bones.size(); ++i) {
    auto &&bone = bones[i];
    Eigen::Vector3f rotaionAxis = Eigen::Vector3f::UnitZ().cross(bone.direction);
    float dotValue = Eigen::Vector3f::UnitZ().dot(bone.direction);
    float crossValue = rotaionAxis.norm();
    rotaionAxis.normalize();
    float theta = atan2(crossValue, dotValue);
    bone.globalFacing = Eigen::AngleAxisf(theta, rotaionAxis);
    bone.globalFacing.scale(Eigen::Vector3f(1.0f, 1.0f, bone.length));
  }
}

Bone *Skeleton::bone(const std::string &name) {
  for (size_t i = 0; i < bones.size(); ++i) {
    if (name == bones[i].name) {
      return &bones[i];
    }
  }
  return nullptr;
}

const Bone *const Skeleton::bone(const std::string &name) const {
  for (size_t i = 0; i < bones.size(); ++i) {
    if (name == bones[i].name) {
      return &bones[i];
    }
  }
  return nullptr;
}

void Skeleton::setModelMatrix(Eigen::Ref<Eigen::Matrix4Xf> modelMatrix) {
  for (size_t i = 0; i < bones.size(); ++i) {
    auto &&bone = bones[i];
    Eigen::Vector3f trans = 0.5f * (bone.startPosition + bone.endPosition);
    Eigen::Affine3f model = bone.globalFacing;
    model.prerotate(bone.rotation).pretranslate(trans);
    modelMatrix.block<4, 4>(0, i * 4) = model.matrix();
  }
}

bool Skeleton::readASFFile(const std::string &filename) {
  std::ifstream input_stream(filename);
  if (!input_stream) {
    std::cerr << "Failed to open " << filename << std::endl;
    return false;
  }
  // ignore header information
  std::string str, keyword;
  while (true) {
    std::getline(input_stream, str);
    if (str.compare(0, 9, ":bonedata") == 0) {
      break;
    }
  }
  // Ignore begin
  input_stream.ignore(1024, '\n');
  bool done = false;
  while (!done) {
    auto &&current_bone = bones.emplace_back();
    while (true) {
      input_stream >> keyword;
      if (keyword == "end") break;
      // finish read bone data, start setup bone hierarchy
      if (keyword == ":hierarchy") {
        done = true;
        bones.pop_back();
        break;
      }
      // id of bone
      if (keyword == "id") {
        input_stream >> current_bone.idx;
        continue;
      }
      // name of the bone
      if (keyword == "name") {
        input_stream >> current_bone.name;
        continue;
      }
      // this line describes the bone's direction vector in global coordinate
      // it will later be converted to local coorinate system
      if (keyword == "direction") {
        input_stream >> current_bone.direction[0] >> current_bone.direction[1] >> current_bone.direction[2];
        continue;
      }
      // length of the bone
      if (keyword == "length") {
        input_stream >> current_bone.length;
        current_bone.length *= _scale;
        continue;
      }
      // this line describes the orientation of bone's local coordinate
      // system relative to the world coordinate system
      if (keyword == "axis") {
        float rx, ry, rz;
        input_stream >> rx >> ry >> rz;
        rx *= static_cast<float>(EIGEN_PI / 180.0L);
        ry *= static_cast<float>(EIGEN_PI / 180.0L);
        rz *= static_cast<float>(EIGEN_PI / 180.0L);
        current_bone.axis = Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()) *
                            Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()) *
                            Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX());
        continue;
      }
      // this line describes the bone's dof
      if (keyword == "dof") {
        ++_movableBones;
        std::string token;
        current_bone.dof = 0;
        std::getline(input_stream, str);
        std::istringstream iss(str);
        while (iss >> token) {
          if (token.compare(0, 2, "rx") == 0) {
            current_bone.dofrx = true;
            ++current_bone.dof;
          } else if (token.compare(0, 2, "ry") == 0) {
            current_bone.dofry = true;
            ++current_bone.dof;
          } else if (token.compare(0, 2, "rz") == 0) {
            current_bone.dofrz = true;
            ++current_bone.dof;
          } else if (token.compare(0, 2, "tx") == 0) {
            current_bone.doftx = true;
            ++current_bone.dof;
          } else if (token.compare(0, 2, "ty") == 0) {
            current_bone.dofty = true;
            ++current_bone.dof;
          } else if (token.compare(0, 2, "tz") == 0) {
            current_bone.doftz = true;
            ++current_bone.dof;
          } else {
            std::cerr << "Unknown token: " << token << std::endl;
          }
        }
        continue;
      }
      if (keyword == "limits") {
        char c;
        if (current_bone.dofrx) {
          input_stream >> c >> current_bone.rxmin >> current_bone.rxmax >> c;
          current_bone.rxmin *= static_cast<float>(EIGEN_PI / 180.0L);
          current_bone.rxmax *= static_cast<float>(EIGEN_PI / 180.0L);
        }
        if (current_bone.dofry) {
          input_stream >> c >> current_bone.rymin >> current_bone.rymax >> c;
          current_bone.rymin *= static_cast<float>(EIGEN_PI / 180.0L);
          current_bone.rymax *= static_cast<float>(EIGEN_PI / 180.0L);
        }
        if (current_bone.dofrz) {
          input_stream >> c >> current_bone.rzmin >> current_bone.rzmax >> c;
          current_bone.rzmin *= static_cast<float>(EIGEN_PI / 180.0L);
          current_bone.rzmax *= static_cast<float>(EIGEN_PI / 180.0L);
        }
      }
    }
  }
  // skip "begin" line
  input_stream.ignore(1024, '\n');
  input_stream.ignore(1024, '\n');
  // Assign parent/child relationship to the bones
  while (true) {
    // read next line
    std::getline(input_stream, str);
    std::istringstream iss(str);
    iss >> keyword;
    // check if we are done
    if (keyword == "end") break;
    // parse this line, it contains parent followed by children
    Bone *parent = this->bone(keyword);
    while (iss >> keyword) {
      this->setChildrenSibling(parent, this->bone(keyword));
    }
  }
  std::cout << bones.size() << " bones in " << filename << " are read" << std::endl;
  input_stream.close();
  return true;
}

int Skeleton::setChildrenSibling(Bone *parent, Bone *child) {
  if (parent == nullptr) {
    printf("inbord bone is undefined\n");
    return 0;
  } else {
    child->parent = parent;
    // if pParent bone does not have a child
    // set pChild as parent bone child
    if (parent->child == nullptr) {
      parent->child = child;
    } else {
      // if pParent bone already has a child
      // set pChils as pParent bone's child sibling
      parent = parent->child;
      while (parent->sibling != nullptr) {
        parent = parent->sibling;
      }
      parent->sibling = child;
    }
    return 1;
  }
}

/**
 * @brief Transform the direction vector (direction),
 * which is defined in character's global coordinate system in the ASF file,
 * to local coordinate
 */
void Skeleton::rotateLocalCoordinate() {
  for (size_t i = 1; i < bones.size(); ++i) {
    // Transform direction vector into local coordinate system
    bones[i].direction = bones[i].axis.inverse() * bones[i].direction;
  }
}

void Skeleton::computeRotationParent2Child() {
  bones[0].rotationParentCurrent = bones[0].axis;
  // Compute rotationParentCurrent for all other bones
  for (size_t i = 0; i < bones.size(); i++) {
    if (bones[i].child != nullptr) {
      bones[i].child->rotationParentCurrent = bones[i].axis.inverse() * bones[i].child->axis;
      // compute parent child siblings...
      Bone *tmp = bones[i].child->sibling;
      while (tmp != nullptr) {
        tmp->rotationParentCurrent = bones[i].axis.inverse() * tmp->axis;
        tmp = tmp->sibling;
      }
    }
  }
}
