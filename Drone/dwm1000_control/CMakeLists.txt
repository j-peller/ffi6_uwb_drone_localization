cmake_minimum_required(VERSION 3.10)

# Set the project name
project(dwm1000_control)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
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

# Add the source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link any required libraries (if applicable)
# target_link_libraries(${PROJECT_NAME} <library_name>)
