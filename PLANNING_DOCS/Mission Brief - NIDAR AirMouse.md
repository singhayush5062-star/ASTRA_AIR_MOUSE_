## Track 1 - Drone Innovation

## Mission Brief - NIDAR AirMouse

Autonomous GPS-Denied Indoor Search, Mapping & Survivor Localisation Challenge

## Scenario

A building has been severely affected by an earthquake / structural collapse. Several internal passages and rooms are unsafe for immediate entry by rescue personnel because of debris, damaged structures, low visibility, and the risk of further collapse. Initial reports suggest that survivors may be trapped inside different rooms, but their exact locations are unknown.

Since GPS signals are unavailable inside the building, rescue teams need an autonomous aerial system that can enter the structure, navigate through enclosed corridors and rooms, identify survivors, generate a 2D map of the explored indoor area, and tag the precise location of each survivor on the map.

Innovation teams are required to build an autonomous indoor drone system that can enter a covered maze-like building environment from one designated entry point, navigate through multiple passages and room-like sections, locate up to 6 survivors, generate a 2D map of the explored area, tag survivor locations on the map, and exit from a designated exit point.

The goal of the innovation teams is to autonomously navigate an unknown GPS-denied indoor environment, find survivors, and provide rescue teams with a usable 2D map showing the tagged location of each survivor in the shortest possible time.

## 1. Overall Objective

- Teams must deploy an autonomous drone capable of operating in a GPS-denied indoor environment.

- The drone must enter the indoor maze from a designated entry & exit point.

- The drone must navigate through corridors, turns, junctions, and rooms inside the covered arena.

- The system must detect up to 6 survivors (real humans or dummies) placed in different rooms and identify the grid coordinate or grid box where each survivor is located.

- The system must simultaneously generate and continuously display a 2D map of the explored area in the Mission Planner / Ground Control Station while the drone is flying.

- The system must tag each detected survivor's location on the generated map by showing a marker on the map.

- The Mission Planner / Ground Control Station must display a live camera feed from the drone throughout the mission.

- The mission must be completed in the shortest possible time while maintaining safe flight, autonomy, mapping accuracy, and localisation accuracy for survivors.

## 2. Drone Configuration & Weight Constraints


- Use of commercially available market-ready or ready-to-fly complete drone airframes shall not be permitted.

- The combined all-up weight of the deployed drone, including batteries, onboard systems, sensors, communication equipment, payload mechanisms, and any other component, must not exceed 10 kg.

- The drone must be equipped with propeller covers or guards that fully protect the propeller operating area.

## 3. Arena, Grid & Mission Environment

- The mission shall be conducted within a covered, maze-like indoor arena representing corridors, rooms, turns, junctions, and enclosed spaces within a damaged building.

- The total maze/arena size shall not exceed 15 m x 15 m.

- The arena shall be designed on a modular grid structure. The top of the indoor space will be covered with a net.

- Each corridor shall have a uniform clear width of at least 1 metre and a vertical clearance of at least 8 feet to allow safe drone navigation within the maze.

- The top of the arena shall be covered to simulate an indoor / GPS-denied environment.

- The arena shall include the same designated entry point and exit point.

- The standard room size shall be 2 m x 2 m (l x b).

- The height clearance in corridors and all rooms at all times shall be 8 feet.

- Corridor widths and grid dimensions shall remain uniform across the arena.

- The exact internal layout of the maze may not be disclosed to teams before the mission.

## 4. Mission Planning, Execution & Autonomy

- Teams shall be provided a maximum setup time of 5 minutes to position and prepare the drone, Ground Control Station, communication systems and associated equipment before the mission begins.

- The drone must begin the mission from the designated entry point.

- After the mission is initiated, the drone must autonomously:

- enter the maze;

- navigate through the indoor environment;

- identify corridors, turns, junctions, rooms and obstacles;

- detect survivors;

- Identify the grid coordinate or grid box in which each detected survivor is located;

- simultaneously generate and continuously update a 2D map of the explored area during flight;

- tag each detected survivor within the relevant grid coordinate or grid box on the generated map;

- proceed toward the designated exit point;

- Exit the maze safely.


- The maximum permitted flying time shall be 30 minutes, measured from take-off until the drone exits the maze or the mission is terminated.

- The system must not rely on GPS / GNSS-based navigation.

- Any manual control input, path correction, waypoint adjustment, survivor tagging input, or operator-assisted navigation during the mission shall be considered manual intervention.

- The operator shall not use FPV goggles or any separate piloting, navigation, monitoring, or video-viewing device during the mission. All authorised mission supervision and live video viewing must take place only through the designated Mission Planner / Ground Control Station.

- The operator may only start the mission and trigger the safety abort/emergency stop if required.

- All mapping, survivor localisation, data processing and transfer of mission data to the Mission Planner / Ground Control Station must occur during flight. No additional time shall be provided after flying for generating or completing the map, processing mission data, transferring data, or updating the Ground Control Station.

## 5. Indoor Mapping & Mission Planner Requirements

- The Mission Planner / Ground Control Station must display:

- live mission status of the drone;

- live camera feed from the drone;

- a 2D map generated and continuously updated while the drone is flying;

- identified corridors, rooms or sections, wherever technically feasible;

- the grid coordinate or grid box containing each detected survivor;

- tagged locations of detected survivors;

- drone position or estimated drone position within the mapped area;

- mission progress and completion status.

- The 2D map, survivor locations and mission data must be generated, displayed and transferred to the Mission Planner / Ground Control Station during flight. No post-flight time shall be provided for completing the map, transferring data or making any corrections.

- The displayed map should be usable by rescue teams to understand the approximate layout of the indoor environment and proceed toward the tagged survivor locations.

- Any manual drawing of the map, manual tagging of survivors, or operator-assisted correction of survivor locations during the mission shall be considered manual intervention.

## 6. Survivor Detection & Localisation

- Up to 6 survivors shall be placed at different locations within the maze / room-like sections.

- The drone must detect survivors using onboard sensing and processing.

- The system must identify and tag the grid coordinate or grid box in which each detected survivor is located on the generated 2D map.

- Survivor locations must be shown by clearly identifying the relevant grid coordinate or grid box to guide rescue teams to the corresponding room or section.


- Survivor detection, grid localisation and tagging must occur autonomously and be displayed on the continuously generated 2D map during flight.

## 7. Communication & Network Constraint

- The mission shall be designed and executed assuming a no-external-network environment.

- Teams shall not rely on GSM, LTE, 5G, public Wi-Fi, internet connectivity, cloud-based communication, or any external network for drone operations, mission execution, data transfer, or coordination.

- All communication between drones and the Mission Planner/Ground Control Station must be handled through the team’s local communication link or onboard autonomy system.

- The system must operate assuming there is no mobile network or internet connectivity.

- Use of any external network-based communication interface during the mission shall be treated as a violation or manual/external intervention, as applicable.

## 8. Team Deployment & Human Intervention

- A maximum of two team members shall be allowed to set up the drone, Ground Control Station, communication systems and associated equipment during the permitted 5-minute setup period.

- No more than one operator shall supervise the mission through the Ground Control Station.

- No assistance of any kind shall be allowed from any other team member during setup, launch, mission execution, mapping, survivor detection, exit, landing, troubleshooting, or recovery.

- Other team members may be present only as observers and shall not provide verbal, physical, digital, or technical assistance during the mission.

## 9. Launch, Landing & Field Constraints

- The drone must be launched from within a fixed 2 feet x 2 feet launch area.

- No part of the drone shall cross or remain outside the designated launch area before mission start.

- The drone must be sized and configured to operate safely within corridors having a minimum clear width of 1 metre and a minimum vertical clearance of 8 feet.

- The drone must be capable of safely navigating confined indoor spaces without touching walls, ceiling, arena panels, or obstacles.

## 10. Safety & Failsafe Requirements

- The drone must include emergency stop/mission abort capability.

- The drone must include failsafe features for: low battery; loss of command and control link; geofence breach; mission abort; emergency recall.

- The system must allow the operator to safely abort the mission if required.

- The drone must be designed for safe operation inside confined indoor spaces.
