cmake_minimum_required(VERSION 3.18...3.22 FATAL_ERROR)
project(CA2022)
# Executable goes to bin
set(CMAKE_MODULE_PATH "${CA2022_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CA2022_SOURCE_DIR}/bin/$<0:>)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CA2022_SOURCE_DIR}/bin/$<0:>)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CA2022_SOURCE_DIR}/lib/$<0:>)
option(BUILD_SHARED_LIBS "Build shared library" ON)
# Set to Release by default
if (NOT (CMAKE_BUILD_TYPE OR CMAKE_CONFIGURATION_TYPES))
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "RelWithDebInfo" "MinSizeRel" "Debug")
endif()
# Detect some compiler flags
include(CheckCXXCompilerFlag)
include(CheckIPOSupported)
if (NOT COMPILER_FLAG_TEST_COMPLETE)
  check_ipo_supported(RESULT COMPILER_SUPPORT_IPO)
  check_cxx_compiler_flag("-march=native" COMPILER_SUPPORT_MARCH_NATIVE)
  set(COMPILER_FLAG_TEST_COMPLETE TRUE)
endif()
# SIMD support
if (COMPILER_SUPPORT_MARCH_NATIVE)
  add_compile_options("-march=native")
endif()
# Detect SIMD support if using Visual Studio since it doensn't provide -march=native
if (MSVC AND NOT MSVC_SIMD_DETECTED)
  # Detect AVX512
  if (NOT DEFINED AVX512_RUN_RESULT)
    message(STATUS "Checking AVX512")
    check_cxx_compiler_flag("/arch:AVX512" COMPILER_SUPPORT_AVX512)
    try_run(AVX512_RUN_RESULT AVX512_COMPILE_RESULT ${CA2022_BINARY_DIR} ${CA2022_SOURCE_DIR}/cmake/cputest/avx512.cpp)
  endif()
  # Detect AVX2
  if (NOT DEFINED AVX2_RUN_RESULT)
    message(STATUS "Checking AVX2 ...")
    check_cxx_compiler_flag("/arch:AVX2" COMPILER_SUPPORT_AVX2)
    try_run(AVX2_RUN_RESULT AVX2_COMPILE_RESULT ${CA2022_BINARY_DIR} ${CA2022_SOURCE_DIR}/cmake/cputest/avx2.cpp)
  endif()
  # Detect AVX
  if (NOT DEFINED AVX_RUN_RESULT)
    message(STATUS "Checking AVX ...")
    check_cxx_compiler_flag("/arch:AVX" COMPILER_SUPPORT_AVX)
    try_run(AVX_RUN_RESULT AVX_COMPILE_RESULT ${CA2022_BINARY_DIR} ${CA2022_SOURCE_DIR}/cmake/cputest/avx.cpp)
  endif()

  if (AVX512_RUN_RESULT EQUAL 0 AND COMPILER_SUPPORT_AVX512)
    message(STATUS "Your CPU supports AVX512")
    add_compile_options("/arch:AVX512")
  elseif (AVX2_RUN_RESULT EQUAL 0 AND COMPILER_SUPPORT_AVX2)
    message(STATUS "Your CPU supports AVX2")
    add_compile_options("/arch:AVX2")
  elseif (AVX_RUN_RESULT EQUAL 0 AND COMPILER_SUPPORT_AVX)
    message(STATUS "Your CPU supports AVX")
    add_compile_options("/arch:AVX")
  else()
    message(STATUS "Your CPU does not support AVX, will use SSE")
  endif()
  set(MSVC_SIMD_DETECTED TRUE)
endif()
# Link time optimization
if (COMPILER_SUPPORT_IPO)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()
# Some MSVC specific flags
if (MSVC)
  add_compile_options("/permissive-")
  # __cplusplus macro will present true value
  add_compile_options("/Zc:__cplusplus")
  # Visual Studio need this for multithread compile
  add_compile_options("/MP")
  # Use UTF8 charset
  add_compile_options("/utf-8")
endif()
# Some GLFW flags
set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)
set(GLFW_BUILD_DOCS OFF)
set(GLFW_INSTALL OFF)
# Homework
add_subdirectory(src)
# Third party libs
add_subdirectory(extern/eigen)
add_subdirectory(extern/glad)
add_subdirectory(extern/glfw)
add_subdirectory(extern/imgui)
