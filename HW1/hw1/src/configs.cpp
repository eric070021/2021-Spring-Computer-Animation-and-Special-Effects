#include "configs.h"
float mouseMoveSpeed = 0.001f;
float keyboardMoveSpeed = 0.1f;

int windowWidth = 0;
int windowHeight = 0;
int speedMultiplier = 1;

float deltaTime = 1e-4f;
int simulationPerFrame = static_cast<int>(baseSpeed / deltaTime);

float springCoef = 25000.0f;
float damperCoef = 750.0f;
float viscousCoef = 3.4e-4f;

Eigen::Vector4f sphereColor = Eigen::Vector4f(0.28f, 0.65f, 0.8f, 1.0f);
Eigen::Vector4f clothColor = Eigen::Vector4f(0.88f, 0.17f, 0.17f, 1.0f);

bool isSphereColorChange = false;
bool isClothColorChange = false;
bool isDrawingParticles = false;
bool isDrawingStructuralSprings = true;
bool isDrawingShearSprings = true;
bool isDrawingBendSprings = false;
bool isDrawingCloth = false;
bool isPaused = true;
bool isStateSwitched = false;

int currentIntegrator = 0;
