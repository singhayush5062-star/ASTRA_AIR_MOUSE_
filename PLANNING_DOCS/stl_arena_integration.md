# STL Arena Integration & Configuration Guide

This document outlines the integration of `mapdraw.stl` as the primary competition arena model and provides instructions on how to configure its orientation and adjust the drone's spawn placement.

---

## 1. How to Modify Arena Orientation

If the STL mesh appears rotated or inverted in the simulation, you can modify its coordinate offset using two different levels in Gazebo:

### Method A: Model-Level Edit (Recommended)
You can directly change the pose of the geometry link in the custom model description file:
- **File**: [model.sdf](file:///home/ayush/Desktop/NIDAR/simulation/custom_models/nidar_arena/model.sdf)
- **Tag**: Locate the `<link name="link">` tag near the top of the file and adjust its `<pose>`:
  ```xml
  <link name="link">
    <!-- Format: X Y Z Roll Pitch Yaw (Roll/Pitch/Yaw are in radians) -->
    <!-- To rotate 180 degrees (pi radians) around Roll (X-axis): 0 0 0 3.14159 0 0 -->
    <pose>0 0 0 0 0 0</pose>
    ...
  </link>
  ```

### Method B: World-Level Edit
Alternatively, you can apply a transformation offset when the model is loaded into the world:
- **File**: [nidar_competition.world](file:///home/ayush/Desktop/NIDAR/nidar_competition.world)
- **Tag**: Locate the `<include>` block referencing `nidar_arena`:
  ```xml
  <include>
    <name>nidar_arena</name>
    <uri>model://nidar_arena</uri>
    <!-- Format: X Y Z Roll Pitch Yaw -->
    <pose>0 0 0 0 0 0</pose>
  </include>
  ```

---

## 2. How to Modify Drone Spawning Position

The drone's initial spawn coordinates are controlled by the master launch script.

### Method A: Command-Line Arguments (Recommended)
When running [test_takeoff.sh](file:///home/ayush/Desktop/NIDAR/scripts/test_takeoff.sh), you can pass custom coordinates directly:
```bash
# Syntax: ./scripts/test_takeoff.sh [gui] [spawn_x] [spawn_y] [spawn_z] [spawn_yaw]
# Example: Spawn the drone in front of the arena at X=2.0, Y=3.0, yaw=1.57 rad (90 degrees)
./scripts/test_takeoff.sh true 2.0 3.0 0.1 1.57
```

### Method B: Edit Default Parameters inside Script
To permanently change the default landing or takeoff starting coordinates:
- **File**: [test_takeoff.sh](file:///home/ayush/Desktop/NIDAR/scripts/test_takeoff.sh)
- **Lines**: Edit the default values for the following shell variables:
  ```bash
  SPAWN_X=${2:-0.0}   # Default spawn X coordinate
  SPAWN_Y=${3:-0.0}   # Default spawn Y coordinate
  SPAWN_Z=${4:-0.1}   # Default spawn Z coordinate (elevation above ground)
  SPAWN_YAW=${5:-0.0} # Default spawn Heading (Yaw in radians)
  ```
