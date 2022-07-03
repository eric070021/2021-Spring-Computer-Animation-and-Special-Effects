#include "kinematics.h"

#include <algorithm>
#include <stack>
#include <map>

#include "utils.h"
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // You should set these variables:
  //     bone->startPosition = Eigen::Vector3f::Zero();
  //     bone->endPosition = Eigen::Vector3f::Zero();
  //     bone->rotation = Eigen::Quaternionf::Identity();
  // The sample above just set everything to initial state
  // Hint:
  //   1. posture.translations, posture.rotations
  // Note:
  //   1. This function will be called with bone == root bone of the skeleton
  
  std::map<int, bool> visit;
  std::stack<Bone*> q;
  visit[bone->idx] = true;
  bone->startPosition = posture.translations[bone->idx];
  bone->endPosition = posture.translations[bone->idx];
  q.push(bone);

  while (!q.empty()) {
    Bone* t = q.top();
    q.pop();
    if (t->idx != 0) {
      t->startPosition = t->parent->endPosition;
      Eigen::Quaternionf rot = t->rotationParentCurrent * posture.rotations[t->idx];
      for (Bone* itr = t->parent; itr != nullptr; itr = itr->parent) {
        rot = itr->rotationParentCurrent * posture.rotations[itr->idx] * rot;
      }

      Eigen::Vector3f dir = t->direction * t->length;
      t->endPosition = rot.toRotationMatrix() * (dir + posture.translations[t->idx]) + t->startPosition;
      t->rotation = rot;
    }
    for (Bone* itr = t->child; itr != nullptr; itr = itr->sibling) {
      if (visit.find(itr->idx) == visit.end()) {
        visit[itr->idx] = true;
        q.push(itr);
      }
    }
  }
}

Motion motionWarp(const Motion& motion, int oldKeyframe, int newKeyframe) {
  Motion newMotion = motion;
  int totalFrames = static_cast<int>(motion.size());
  int totalBones = static_cast<int>(motion.posture(0).rotations.size());
  int halfFrames = totalFrames / 2;
  int warpFrames = 300;

  for (int i = 0; i < totalFrames; ++i) {
    if (i < warpFrames) {
      int l = halfFrames * i / warpFrames;
      int h = l + 1;
      double ratio = double(halfFrames * i / warpFrames) - l; // align for slerp
      for (int j = 0; j < totalBones; ++j) {
        newMotion.posture(i).translations[j] =
            ratio * motion.posture(h).translations[j] + (1 - ratio) * motion.posture(l).translations[j];
        newMotion.posture(i).rotations[j] =
            motion.posture(l).rotations[j].slerp(ratio, motion.posture(h).rotations[j]);
      }
    } else if (i >= warpFrames && i < totalFrames - 1) {
      int l = (totalFrames - halfFrames) * (i - warpFrames) / (totalFrames - warpFrames) + halfFrames;
      int h = l + 1;
      double ratio = double((totalFrames - halfFrames) * (i - warpFrames) / (totalFrames - warpFrames)) +
          halfFrames - l; // align for slerp
      for (int j = 0; j < totalBones; ++j) {
        newMotion.posture(i).translations[j] =
            ratio * motion.posture(h).translations[j] + (1 - ratio) * motion.posture(l).translations[j];
        newMotion.posture(i).rotations[j] =
            motion.posture(l).rotations[j].slerp(ratio, motion.posture(h).rotations[j]);
      }
    }
  }
  return newMotion;
}

Motion motionBlend(const Motion& motionA, const Motion& motionB) {
  Motion newMotion;
  constexpr int blendFrameCount = 20;
  constexpr float blendFactor = 1.0f / blendFrameCount;
  constexpr int matchRange = 10;
  float difference[matchRange] = {};
  // TODO (Bonus)
  // motionA: |--------------|--matchRange--|--blendFrameCount--|
  // motionB:                               |--blendFrameCount--|--------------|
  // The starting frame of `blendFrameCount` can be in `matchRange`
  // Hint:
  //   1. Find motionB's starting posture
  //   2. Match it with the minimum cost posture in `matchRange`
  //   3. Find to translation and rotation offset between matched motionA and motionB's start
  //   4. Begin from the matched frame, blend `blendFrameCount` of frames,
  //      with a blendFactor from 1 / `blendFrameCount` to 1
  //   5. Add remaining motionB to newMotion
  // Note:
  //   1. The offset found in 3 should apply to 4 and 5
  //   2. A simple cost function is translation offsets between two posture.
  //   3. A better one considered both translations and rotations.
  //   4. Your animation should smoothly change from motionA to motionB.
  //   5. You can adjust those `constexpr`s above by yourself if you need.

  // Write your code here
  return newMotion;
}
