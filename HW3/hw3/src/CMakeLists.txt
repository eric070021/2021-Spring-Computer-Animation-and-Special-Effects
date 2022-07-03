project(HW3 C CXX)

set(HW3_SOURCE
  ${HW3_SOURCE_DIR}/buffer.cpp
  ${HW3_SOURCE_DIR}/camera.cpp
  ${HW3_SOURCE_DIR}/configs.cpp
  ${HW3_SOURCE_DIR}/cylinder.cpp
  ${HW3_SOURCE_DIR}/glcontext.cpp
  ${HW3_SOURCE_DIR}/gui.cpp
  ${HW3_SOURCE_DIR}/kinematics.cpp
  ${HW3_SOURCE_DIR}/motion.cpp
  ${HW3_SOURCE_DIR}/posture.cpp
  ${HW3_SOURCE_DIR}/shader.cpp
  ${HW3_SOURCE_DIR}/skeleton.cpp
  ${HW3_SOURCE_DIR}/sphere.cpp
  ${HW3_SOURCE_DIR}/utils.cpp
  ${HW3_SOURCE_DIR}/vertexarray.cpp
)

set(HW3_INCLUDE_DIR ${HW3_SOURCE_DIR}/../include)

add_executable(HW3 ${HW3_SOURCE} ${HW3_SOURCE_DIR}/main.cpp)
target_include_directories(HW3 PRIVATE ${HW3_INCLUDE_DIR})

add_dependencies(HW3 glad glfw eigen)
# Can include glfw and glad in arbitrary order
target_compile_definitions(HW3 PRIVATE GLFW_INCLUDE_NONE)
# More warnings
if (NOT MSVC)
  target_compile_options(HW3
    PRIVATE "-Wall"
    PRIVATE "-Wextra"
    PRIVATE "-Wpedantic"
  )
endif()
# Prefer std c++20, at least need c++17 to compile
set_target_properties(HW3 PROPERTIES
  CXX_STANDARD 20
  CXX_EXTENSIONS OFF
)

target_link_libraries(HW3
  PRIVATE glad
  PRIVATE glfw
  PRIVATE eigen
  PRIVATE dearimgui
)
