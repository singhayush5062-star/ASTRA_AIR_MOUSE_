cmake_minimum_required(VERSION 3.0.2)
project(nidar_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Survivor.msg
  SurvivorArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)
