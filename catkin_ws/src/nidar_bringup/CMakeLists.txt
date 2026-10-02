cmake_minimum_required(VERSION 3.0.2)
project(nidar_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/test_takeoff.sh
  scripts/cpu_repin_loop.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
