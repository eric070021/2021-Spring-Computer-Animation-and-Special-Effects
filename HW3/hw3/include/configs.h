#pragma once

#include <Eigen/Core>
// constants
inline constexpr int cylinderSectors = 72;
inline constexpr float cylinderHeight = 1.0f;
inline constexpr float cylinderRadius = 0.1f;

inline constexpr int sphereSlice = 36;
inline constexpr int sphereStack = 18;
// variables

extern int windowWidth;
extern int windowHeight;

extern float mouseMoveSpeed;
extern float keyboardMoveSpeed;

extern int startBoneID;
extern int endBoneID;

extern bool isPlaying;
extern bool isIKChanged;
extern bool resetTrigger;

extern Eigen::Vector3f target;
