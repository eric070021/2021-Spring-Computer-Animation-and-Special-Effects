#pragma once

#include "bone.h"
#include "motion.h"
#include "posture.h"

void forwardKinematics(const Posture& posture, Bone* root);
void inverseKinematics(const Eigen::Vector3f& target, Bone* start, Bone* end, Posture& posture);
