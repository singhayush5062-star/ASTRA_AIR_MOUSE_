cmake_minimum_required(VERSION 3.0.2)
project(nidar_safety)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/flight_envelope_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
