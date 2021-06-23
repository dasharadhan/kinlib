cmake_minimum_required(VERSION 3.0.2)
project(kinlib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

set(KINLIB_TEST_RESOURCES_DIR "${CMAKE_CURRENT_LIST_DIR}/files")
configure_file("include/${PROJECT_NAME}_test/config.h.in" "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}_test/config.h")
set(KINLIB_TEST_INCLUDE_DIR "${CATKIN_DEVEL_PREFIX}/include/")

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    trajectory_msgs
    urdf
    kdl_parser
)

find_package(Boost REQUIRED system filesystem)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    trajectory_msgs
    urdf
    kdl_parser
  DEPENDS
    EIGEN3
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/DualQuat.cpp
  src/manipulator.cpp
  src/kinlib_kinematics.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/src
)

if(CATKIN_ENABLE_TESTING)
  include_directories(${KINLIB_TEST_INCLUDE_DIR})
  catkin_add_gtest(kinlib_test test/kinlib_test.cpp)
  target_link_libraries(kinlib_test ${PROJECT_NAME} ${catkin_LIBRARIES} armadillo)
endif()
