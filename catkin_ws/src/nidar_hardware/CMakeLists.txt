cmake_minimum_required(VERSION 3.0.2)
project(nidar_hardware)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/livox_bridge.py
  scripts/rangefinder_node.py
  scripts/camera_publisher.py
  scripts/hw_static_tf.py
  scripts/mission_commander.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
