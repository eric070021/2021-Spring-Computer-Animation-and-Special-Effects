#pragma once

// constants
constexpr int cylinderSectors = 72;
constexpr float cylinderHeight = 1.0f;
constexpr float cylinderRadius = 0.1f;

// variables

extern int windowWidth;
extern int windowHeight;

extern float mouseMoveSpeed;
extern float keyboardMoveSpeed;

extern bool isSimulating;
extern bool isMotionChanged;

extern int maxFrame;
extern int currentFrame;
extern int currentMotion;
