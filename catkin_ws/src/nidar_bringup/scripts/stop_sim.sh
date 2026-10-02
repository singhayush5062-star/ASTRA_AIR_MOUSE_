#!/bin/bash
# Stop every process a simulation run starts (Gazebo, PX4, MAVROS, FAST-LIO, FUEL, the guard and
# the mission layer), including the ones `killall roslaunch` would orphan. Used at the start of
# every run by test_takeoff.sh (clean slate) and by the GCS RESET button (nidar_gcs), so there is
# exactly one kill list to keep complete.
#
# rosbag needs SIGINT (not SIGKILL) to write its index, so any recorder left over from a killed
# run is interrupted first and given a moment to close its bag.
pkill -INT -f "rosbag record" 2>/dev/null || true
pkill -INT -f "lib/rosbag/record" 2>/dev/null || true
sleep 1
killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
pkill -f flight_envelope_guard.py 2>/dev/null || true
pkill -f relay_odometry.py 2>/dev/null || true
pkill -f exploration_node 2>/dev/null || true
pkill -f traj_server 2>/dev/null || true
pkill -f waypoint_generator 2>/dev/null || true
pkill -f fast_lio 2>/dev/null || true
pkill -f FAST_LIO 2>/dev/null || true
pkill -f cpu_repin_loop.sh 2>/dev/null || true
# rviz and the telemetry logger were missing from this list, and both outlive a killed run:
# rviz is started by nidar_mapping.launch (rviz:=$GUI_ARG) and is not one of the `killall` names
# above, while mission_telemetry_logger.py runs in the foreground of the previous invocation.
# A surviving rviz idles at ~290% CPU and a surviving FAST-LIO publishes a SECOND, conflicting
# solution onto /Fast_LIO/odometry, which makes the next run's odometry jump between two
# estimates and look like a SLAM divergence. Observed on 2026-09-04: a re-run inherited 3 gzserver
# / 3 px4 / 2 fastlio_mapping processes and diverged to (-26, 56) within 30 s of takeoff.
pkill -f rviz 2>/dev/null || true
pkill -f mission_telemetry_logger.py 2>/dev/null || true
pkill -f flightlog/record.py 2>/dev/null || true
pkill -f flightlog/watchdog.py 2>/dev/null || true
# mission_manager.py was entry_detection_module.py before the package split; keep killing the old
# name too so an orphan from a pre-split checkout cannot survive into this run.
pkill -f mission_manager.py 2>/dev/null || true
pkill -f entry_detection_module.py 2>/dev/null || true
# robot_state_publisher / static_transform_publisher are started by nidar_mapping.launch and are
# not among the `killall` names either, so they accumulate across runs -- five of them were found
# alive with no simulation running.
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f static_transform_publisher 2>/dev/null || true
# The mission-layer Python nodes (started by mission_only.launch) were missing too. `killall -9
# roslaunch` above kills only the launcher, which orphans its children rather than stopping them,
# so a previous run's detector / slicer / overlay / coverage reporter kept running alongside the
# next run's copies (found alive 55 min after their run ended on 2026-10-02).
pkill -f survivor_detector.py 2>/dev/null || true
pkill -f coverage_reporter.py 2>/dev/null || true
pkill -f map_2d_slicer.py 2>/dev/null || true
pkill -f lidar_map_2d.py 2>/dev/null || true
pkill -f grid_visualizer.py 2>/dev/null || true
# Give the SIGKILLs time to land before spawning replacements -- `killall` returns immediately and
# gzserver in particular (a /bin/sh wrapper plus a forked child) can outlive the call by a second.
sleep 3
