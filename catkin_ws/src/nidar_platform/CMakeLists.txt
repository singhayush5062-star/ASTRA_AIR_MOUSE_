cmake_minimum_required(VERSION 3.0.2)
project(nidar_platform)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/relay_odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/build_px4.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
