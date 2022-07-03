#include "configs.h"
float mouseMoveSpeed = 0.001f;
float keyboardMoveSpeed = 0.1f;

int windowWidth = 0;
int windowHeight = 0;

int startBoneID = 11;
int endBoneID = 29;

bool isPlaying = false;
bool isIKChanged = true;
bool resetTrigger = false;

Eigen::Vector3f target = Eigen::Vector3f(-0.0692501f, 3.85358f, -1.63441f);
