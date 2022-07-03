#pragma once
#include <Eigen/Core>

// constants
inline constexpr int particlesPerEdge = 25;
inline constexpr int clothWidth = 2;
inline constexpr int clothHeight = 2;
inline constexpr float particleMass = 1.0f;
inline constexpr float sphereDensity = 1e3f;
inline constexpr float baseSpeed = 1e-3f;

inline constexpr int sphereSlice = 36;
inline constexpr int sphereStack = 18;

// variables

extern int windowWidth;
extern int windowHeight;
extern int speedMultiplier;

extern float mouseMoveSpeed;
extern float keyboardMoveSpeed;

extern float deltaTime;
extern int simulationPerFrame;

extern float springCoef;
extern float damperCoef;
extern float viscousCoef;

extern Eigen::Vector4f sphereColor;
extern Eigen::Vector4f clothColor;

extern bool isSphereColorChange;
extern bool isClothColorChange;
extern bool isDrawingParticles;
extern bool isDrawingStructuralSprings;
extern bool isDrawingShearSprings;
extern bool isDrawingBendSprings;
extern bool isDrawingCloth;
extern bool isPaused;
extern bool isStateSwitched;

extern int currentIntegrator;
