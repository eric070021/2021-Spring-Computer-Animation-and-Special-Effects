#include "motion.h"

#include <fstream>
#include <iostream>

#include "utils.h"

Motion::Motion(const std::string &amc_file, const Skeleton &skeleton) noexcept {
  _posture.reserve(1024);
  if (!this->readAMCFile(amc_file, skeleton)) {
    std::cerr << "Error in reading AMC file, this object is not initialized!" << std::endl;
    std::cerr << "You can call readAMCFile() to initialize again" << std::endl;
    _posture.resize(0);
  }
}

bool Motion::readAMCFile(const std::string &filename, const Skeleton &skeleton) {
  // Open AMC file
  std::ifstream input_stream(filename);
  // Check if file successfully opened
  if (!input_stream) {
    std::cerr << "Failed to open " << filename << std::endl;
    return false;
  }
  // There are (NUM_BONES_IN_ASF_FILE - 2) moving bones and 2 dummy bones (lhipjoint and rhipjoint)
  int movable_bones = skeleton.movableBoneNum();
  // Ignore header
  input_stream.ignore(1024, '\n');
  input_stream.ignore(1024, '\n');
  input_stream.ignore(1024, '\n');
  int frame_num;
  std::string bone_name;
  while (input_stream >> frame_num) {
    auto &&current_posture = _posture.emplace_back(skeleton.size());
    for (int i = 0; i < movable_bones; ++i) {
      input_stream >> bone_name;
      const Bone &bone = *(skeleton.bone(bone_name));
      int bone_idx = bone.idx;
      float rx = 0, ry = 0, rz = 0;
      Eigen::Vector3f bone_translation = Eigen::Vector3f::Zero();
      if (bone.doftx) input_stream >> bone_translation[0];
      if (bone.dofty) input_stream >> bone_translation[1];
      if (bone.doftz) input_stream >> bone_translation[2];
      if (bone.dofrx) input_stream >> rx;
      if (bone.dofry) input_stream >> ry;
      if (bone.dofrz) input_stream >> rz;
      rx *= static_cast<float>(EIGEN_PI / 180.0L);
      ry *= static_cast<float>(EIGEN_PI / 180.0L);
      rz *= static_cast<float>(EIGEN_PI / 180.0L);
      current_posture.eulerAngle[bone_idx] = Eigen::Vector3f(rx, ry, rz);
      current_posture.rotations[bone_idx] = Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()) *
                                            Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()) *
                                            Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX());
      current_posture.translations[bone_idx] = std::move(bone_translation);
      if (bone_idx == 0) current_posture.translations[bone_idx] *= skeleton.scale();
    }
  }
  input_stream.close();
  std::cout << frame_num << " frames in " << filename << " are read" << std::endl;
  return true;
}
