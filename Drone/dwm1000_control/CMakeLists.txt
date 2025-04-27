cmake_minimum_required(VERSION 3.10)

# Set the project name
project(dwm1000_control)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set build types
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type (Debug or Release)")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release)

# Add compiler flags for Debug and Release
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Add the include directories
include_directories(${CMAKE_SOURCE_DIR}/extern)
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/../../shared/inc)

# Add the source files
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE COORDS_SOURCES "../coords_calc/*.cpp")

# 
find_package(Eigen3 3.4 REQUIRED)

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${COORDS_SOURCES})

# Link the Eigen3 library
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen gpiod)



# Enable testing
#enable_testing()

# Google Test Integration
#include(FetchContent)
#FetchContent_Declare(
#  googletest
#  GIT_REPOSITORY https://github.com/google/googletest.git
#  GIT_TAG        v1.16.0  # You can change the tag to the latest stable release
#)
#FetchContent_MakeAvailable(googletest)

# Include directories for Google Test
#include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# Add a test executable for unit tests
#add_executable(DW1000TimeTests "test/dw1000_time_test.cpp" "src/dw1000_time.cpp")

# Link the test executable with Google Test and the project source
#target_link_libraries(DW1000TimeTests gtest gtest_main)

# Run tests after the build
#add_test(NAME DW1000TimeTests COMMAND DW1000TimeTests)
