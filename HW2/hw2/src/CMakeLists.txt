project(HW2 C CXX)

set(HW2_SOURCE
  ${HW2_SOURCE_DIR}/buffer.cpp
  ${HW2_SOURCE_DIR}/camera.cpp
  ${HW2_SOURCE_DIR}/configs.cpp
  ${HW2_SOURCE_DIR}/cylinder.cpp
  ${HW2_SOURCE_DIR}/glcontext.cpp
  ${HW2_SOURCE_DIR}/gui.cpp
  ${HW2_SOURCE_DIR}/kinematics.cpp
  ${HW2_SOURCE_DIR}/motion.cpp
  ${HW2_SOURCE_DIR}/posture.cpp
  ${HW2_SOURCE_DIR}/shader.cpp
  ${HW2_SOURCE_DIR}/skeleton.cpp
  ${HW2_SOURCE_DIR}/utils.cpp
  ${HW2_SOURCE_DIR}/vertexarray.cpp
)

set(HW2_INCLUDE_DIR ${HW2_SOURCE_DIR}/../include)

add_executable(HW2 ${HW2_SOURCE} ${HW2_SOURCE_DIR}/main.cpp)
target_include_directories(HW2 PRIVATE ${HW2_INCLUDE_DIR})

add_dependencies(HW2 glad glfw eigen)
# Can include glfw and glad in arbitrary order
target_compile_definitions(HW2 PRIVATE GLFW_INCLUDE_NONE)
# More warnings
if (NOT MSVC)
  target_compile_options(HW2
    PRIVATE "-Wall"
    PRIVATE "-Wextra"
    PRIVATE "-Wpedantic"
  )
endif()
# Prefer std c++20, at least need c++17 to compile
set_target_properties(HW2 PROPERTIES
  CXX_STANDARD 20
  CXX_EXTENSIONS OFF
)

target_link_libraries(HW2
  PRIVATE glad
  PRIVATE glfw
  PRIVATE eigen
  PRIVATE dearimgui
)
