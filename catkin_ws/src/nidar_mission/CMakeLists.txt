cmake_minimum_required(VERSION 3.0.2)
project(nidar_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  mavros_msgs
  quadrotor_msgs
  nidar_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs mavros_msgs quadrotor_msgs nidar_msgs
)

catkin_install_python(PROGRAMS
  scripts/mission_manager.py
  scripts/mission_telemetry_logger.py
  scripts/coverage_reporter.py
  scripts/map_2d_slicer.py
  scripts/grid_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
