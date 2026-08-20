cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# ------------------------------------------------------------
# NLOpt dependency discovery
# ------------------------------------------------------------

find_path(NLOPT_INCLUDE_DIR
  NAMES nlopt.hpp
  PATHS
    /usr/include
    /usr/local/include
)

find_library(NLOPT_LIBRARY
  NAMES nlopt
  PATHS
    /usr/lib/x86_64-linux-gnu
    /lib/x86_64-linux-gnu
    /usr/lib
    /lib
    /usr/local/lib
)

if(NOT NLOPT_INCLUDE_DIR)
  message(FATAL_ERROR "NLOpt header <nlopt.hpp> not found")
endif()

if(NOT NLOPT_LIBRARY)
  message(FATAL_ERROR "NLOpt library libnlopt.so not found")
endif()

message(STATUS "NLOpt include directory: ${NLOPT_INCLUDE_DIR}")
message(STATUS "NLOpt library: ${NLOPT_LIBRARY}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  active_perception
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_opt
  CATKIN_DEPENDS plan_env active_perception
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIR}
)

add_library(bspline_opt
  src/bspline_optimizer.cpp
)

target_link_libraries(bspline_opt
  ${catkin_LIBRARIES}
  ${NLOPT_LIBRARY}
)
