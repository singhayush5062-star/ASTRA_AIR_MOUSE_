cmake_minimum_required(VERSION 3.0.2)
project(nidar_gcs)

find_package(catkin REQUIRED)

catkin_package()

# The web UI (frontend/) and the FastAPI backend (backend/) are not built by catkin: they have
# their own toolchains, set up by scripts/setup_gcs.sh. Only the ROS bridge is a ROS program.
catkin_install_python(PROGRAMS
  scripts/gcs_ros_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/setup_gcs.sh scripts/start_gcs.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
