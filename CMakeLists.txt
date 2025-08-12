cmake_minimum_required(VERSION 3.5)
project(kinlib)

# Set CXX Standard for the project
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

set(KINLIB_RESOURCES_DIR "${CMAKE_CURRENT_LIST_DIR}/files/")
configure_file("include/${PROJECT_NAME}/${PROJECT_NAME}_resources.h.in" "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_resources.h")

# Control where libraries and executables are placed during the build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

#find_package(Boost REQUIRED system filesystem)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/DualQuat.cpp
  src/manipulator.cpp
  src/kinlib_kinematics.cpp
  src/motion_planning.cpp
  src/nullSpace_kinematics.cpp
  src/robot_parameter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h")

install(FILES ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_resources.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

# add_executable(kinlib_example src/example.cpp)
# target_include_directories(kinlib_example PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
# 	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# target_link_libraries(kinlib_example ${PROJECT_NAME})

# add_executable(panda_example src/panda_example.cpp)
# target_include_directories(panda_example PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
# 	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# target_link_libraries(panda_example ${PROJECT_NAME})

# add_executable(panda_ScLERP src/panda_ScLERP.cpp)
# target_include_directories(panda_ScLERP PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
# 	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# target_link_libraries(panda_ScLERP ${PROJECT_NAME})

# add_executable(panda_pickPlace src/panda_pickPlace.cpp)
# target_include_directories(panda_pickPlace PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
# 	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# target_link_libraries(panda_pickPlace ${PROJECT_NAME})

# add_executable(test_ScLERP src/test_ScLERP.cpp)
# target_include_directories(test_ScLERP PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
# 	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# target_link_libraries(test_ScLERP ${PROJECT_NAME})


# add_executable(buildWall src/buildWall.cpp)
# target_include_directories(buildWall PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
# 	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# target_link_libraries(buildWall ${PROJECT_NAME})

# add_executable(VR_buildWall src/VR_buildWall.cpp)
# target_include_directories(VR_buildWall PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
# 	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# target_link_libraries(VR_buildWall ${PROJECT_NAME})

# add_executable(VR_curveWall src/VR_curveWall.cpp)
# target_include_directories(VR_curveWall PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
# 	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# target_link_libraries(VR_curveWall ${PROJECT_NAME})

add_executable(celling_tile_build src/celling_tile_build.cpp)
target_include_directories(celling_tile_build PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(celling_tile_build ${PROJECT_NAME})

# add_executable(middle_null_space_test src/middle_null_space_test.cpp)
# target_include_directories(middle_null_space_test PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
# 	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# target_link_libraries(middle_null_space_test ${PROJECT_NAME})


add_executable(null_space_softlimit src/null_space_softlimit.cpp)
target_include_directories(null_space_softlimit PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(null_space_softlimit ${PROJECT_NAME})

add_executable(verify_null_space src/verify_null_space.cpp)
target_include_directories(verify_null_space PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(verify_null_space ${PROJECT_NAME})

add_executable(celling_tile_debug src/celling_tile_debug.cpp)
target_include_directories(celling_tile_debug PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(celling_tile_debug ${PROJECT_NAME})