cmake_minimum_required(VERSION 3.0.2)
project(kinlib)

# Set CXX Standard for the project
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the output folder for executables
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED system filesystem)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/DualQuat.cpp
  src/manipulator.cpp
  src/kinlib_kinematics.cpp
)
