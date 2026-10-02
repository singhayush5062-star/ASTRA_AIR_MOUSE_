cmake_minimum_required(VERSION 3.0.2)
project(nidar_qa)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/analyze_exploration.py
  scripts/map_accuracy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/verify_fix_parity.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
