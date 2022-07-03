cmake_minimum_required(VERSION 3.18.0)

project(DEARIMGUI
  VERSION         1.87
  DESCRIPTION     "Bloat-free Graphical User interface for C++ with minimal dependencies "
  HOMEPAGE_URL    "https://github.com/ocornut/imgui"
  LANGUAGES       CXX
)

set(DEARIMGUI_SOURCE
  ${PROJECT_SOURCE_DIR}/src/imgui.cpp
  ${PROJECT_SOURCE_DIR}/src/imgui_demo.cpp
  ${PROJECT_SOURCE_DIR}/src/imgui_draw.cpp
  ${PROJECT_SOURCE_DIR}/src/imgui_impl_glfw.cpp
  ${PROJECT_SOURCE_DIR}/src/imgui_impl_opengl3.cpp
  ${PROJECT_SOURCE_DIR}/src/imgui_tables.cpp
  ${PROJECT_SOURCE_DIR}/src/imgui_widgets.cpp
)

add_library(dearimgui OBJECT ${DEARIMGUI_SOURCE})
add_dependencies(dearimgui glfw glad)

target_compile_definitions(dearimgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_CUSTOM)
target_compile_features(dearimgui PRIVATE cxx_std_17)
target_link_libraries(dearimgui PRIVATE glfw glad)
target_include_directories(dearimgui PUBLIC ${PROJECT_SOURCE_DIR}/include)
