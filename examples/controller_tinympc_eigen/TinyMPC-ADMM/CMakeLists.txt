##############################
# Preamble
##############################
cmake_minimum_required(VERSION 3.2)
list(APPEND CMAKE_MESSAGE_CONTEXT tinympc)
project(TinyMPC VERSION 0.1 LANGUAGES C CXX)

##############################
# Project wide
##############################
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS NO)

include(CMakePrintHelpers)
include(GNUInstallDirs)
# include(Functions)
include(FetchContent)

# Add CPM Dependency Manager
include(FindCPM)

# Handle default build type
set(TINY_DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified. Setting CMAKE_BUILD_TYPE to ${TINY_DEFAULT_BUILD_TYPE}")
  set(CMAKE_BUILD_TYPE ${TINY_DEFAULT_BUILD_TYPE} CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

# Enable testing
option(TINY_BUILD_TESTS "Build tests for TinyMPC" ON)

# Enable clang-tidy analysis
option(TINY_CLANG_TIDY "Run clang-tidy analyzer on the source code." OFF)

# Floating point precision
set(TINY_FLOAT float CACHE STRING "Floating point precision (float,double).")

# Build examples
option(TINY_BUILD_EXAMPLES "Build examples for TinyMPC." ON)

# Build with -march=native
option(TINY_VECTORIZE "Compile with -march=native" ON)

# Build with extra information for profiling
option(TINY_PROFILE "Compile with -pg" OFF)

##############################
# Dependencies
##############################

# slap
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
FetchContent_Declare(slap
  GIT_REPOSITORY https://github.com/bjack205/slap
  GIT_TAG main
  )
set(SLAP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SLAP_VECTORIZE ${TINY_VECTORIZE} CACHE BOOL "" FORCE)
set(SLAP_FLOAT ${TINY_FLOAT} CACHE STRING "" FORCE)
FetchContent_MakeAvailable(slap)
endif ()

# Google Test
# if (TINY_BUILD_TESTS)
#   CPMAddPackage(
#       NAME googletest
#       GITHUB_REPOSITORY google/googletest
#       VERSION 1.13.0
#       DOWNLOAD_EXTRACT_TIMESTAMP
#       OPTIONS
#       "INSTALL_GTEST OFF"
#       "gtest_force_shared_crt ON"
#   )
#   add_library(gtest::gtest ALIAS gtest_main)
#   enable_testing()
#   include(GoogleTest)
#   include(CTest)
# endif ()

##############################
# Main build targets
##############################
# Compile options
if (NOT WIN32)
  add_compile_options(-Wall -Wextra -pedantic)
  # add_compile_options(-Wno-error=unknown-pragmas
  #                     -Wformat=2 -Wno-unused-parameter -Wshadow 
  #                     -Wwrite-strings -Wstrict-prototypes -Wold-style-definition 
  #                     -Wredundant-decls -Wnested-externs -Wmissing-include-dirs)
  if (TINY_VECTORIZE)
    add_compile_options(-march=native)
  endif()
  # if (TINY_PROFILE)
  #   add_compile_options(-pd)
  # endif()
endif()

if (CMAKE_C_COMPILER_ID MATCHES "GNU")
  add_compile_options(-Wjump-misses-init -Wlogical-op)
endif()  

# TODO: Future options to optimize library for specific use of constraints
# option(SET_GOAL_CONSTRAINT "Set goal constraint" OFF)
# if (SET_GOAL_CONSTRAINT)
#   add_definitions(-DGOAL_CONSTRAINT)
# endif()
# option(SET_INPUT_CONSTRAINT "Set input constraints" ON)
# if (SET_INPUT_CONSTRAINT)
#   add_definitions(-DINPUT_CONSTRAINT)
# endif()
# option(SET_STATE_CONSTRAINT "Set state constraints" ON)
# if (SET_STATE_CONSTRAINT)
#   add_definitions(-DSTATE_CONSTRAINT)
# endif()

# Make all includes relative to src/ directory
include_directories(${PROJECT_SOURCE_DIR}/src)

# Build source files
add_subdirectory(src/tinympc)

##############################
# Tests
##############################
if (TINY_BUILD_TESTS)
  add_subdirectory(tests)
endif()                 

##############################
# Examples
##############################
if (TINY_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()