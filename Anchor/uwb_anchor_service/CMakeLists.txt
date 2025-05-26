cmake_minimum_required(VERSION 3.10)

# Set the project name
project(uwb_anchor_service)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set build types
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type (Debug or Release)")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release)

# Add macro definition when in Debug mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(DEBUG=1)
endif()

# Add compiler flags for Debug and Release
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Add the include directories
include_directories(${CMAKE_SOURCE_DIR}/extern)
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/../../shared/inc)
include_directories(${CMAKE_SOURCE_DIR}/../../Drone/dwm1000_control/inc)

# Add the source files
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE DWM1000_CTRL_SOURCES 
    "../../Drone/dwm1000_control/src/dw1000_time.cpp"
    "../../Drone/dwm1000_control/src/dwm1000_ctrl.cpp"
    "../../Drone/dwm1000_control/src/dwm1000_ranging.cpp"
    "../../Drone/dwm1000_control/src/dwm1000_ranging_anchor.cpp"
    "../../Drone/dwm1000_control/src/dw1000_modes.cpp"
    "../../Drone/dwm1000_control/src/ws_logger.cpp"
    )

# 
find_package(Eigen3 3.4 REQUIRED)
find_library(GPIOD_LIBRARY gpiod)
find_package(PkgConfig REQUIRED)

# Find inih via pkg-config
pkg_check_modules(INIReader REQUIRED IMPORTED_TARGET INIReader)
pkg_check_modules(LIB_WEBSOCKETS REQUIRED libwebsockets)

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${DWM1000_CTRL_SOURCES})

# 
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen gpiod PkgConfig::INIReader ${LIB_WEBSOCKETS_LIBRARIES})