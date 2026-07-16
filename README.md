# Autonomous Mobile Robot System

ROS 2 Jazzy + Gazebo Harmonic workspace for a custom mobile robot that can:

- run in simulation inside a custom Gazebo arena
- build a 2D map with SLAM or a simpler custom mapper
- save and reuse maps
- execute area coverage using a boustrophedon planner
- visualize everything in RViz
- move the robot manually or by a planned coverage path

## What This Project Contains

The workspace is split into two main packages:

- `robot_description`
  - custom URDF/Xacro robot model
  - Gazebo world / arena definition
  - launch files for simulation, SLAM, and coverage
  - Nav2 and SLAM configuration files
  - RViz configuration files
  - Gazebo-ROS bridge setup

- `coverage_planner`
  - `boustrophedon_planner.py`: the main coverage planner and direct motion controller
  - `simple_mapper.py`: a simple occupancy grid mapper alternative to SLAM

## High-Level Architecture

The runtime flow is:

1. `gazebo.launch.py` starts the robot inside Gazebo.
2. `slam.launch.py` or `simple_mapper` builds a map from `/scan`.
3. The resulting map is saved to a YAML + image pair.
4. `coverage.launch.py` loads the saved map and starts the coverage planner.
5. `boustrophedon_planner.py` creates zigzag waypoints and publishes `/cmd_vel` directly.
6. `ros_gz_bridge` sends the command to Gazebo so the robot moves in simulation.

## Features

This project can do the following:

- spawn a custom differential-drive robot in Gazebo
- publish robot state and TF frames
- bridge Gazebo topics to ROS 2 topics
- perform SLAM mapping with `slam_toolbox`
- perform simple occupancy-grid mapping with `simple_mapper`
- save maps for later use
- run Nav2 infrastructure for map loading and costmaps
- perform boustrophedon area coverage
- draw coverage polygons in RViz
- accept coverage commands such as `start`, `clear`, `cancel`, `finish_area`, and `exclude_area`
- navigate directly to a goal pose from RViz inside the planner node

## Requirements

You need:

- Ubuntu 24.04 with ROS 2 Jazzy
- Gazebo Harmonic
- `colcon`
- `rosdep`
- `xterm` for the teleop window launched by `slam.launch.py`

If you run inside a container or Distrobox, make sure the ROS and Gazebo environments are available there.

## Build and Install

From the root of the workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

You must source the workspace in every new terminal before running the launch files.

## Package Layout

### `robot_description`

Important files:

- `launch/gazebo.launch.py` - starts Gazebo, the robot, and the Gazebo-ROS bridge
- `launch/slam.launch.py` - starts SLAM or the simple mapper, RViz, and teleop
- `launch/coverage.launch.py` - starts Nav2 bringup, the coverage planner, and RViz
- `config/nav2_params.yaml` - Nav2 parameters
- `config/slam_toolbox.yaml` - SLAM Toolbox parameters
- `urdf/turtlebot_custom.urdf.xacro` - custom robot model and Gazebo plugins
- `worlds/arena.sdf` - custom arena world
- `rviz/*.rviz` - RViz layouts for mapping and coverage

### `coverage_planner`

Important files:

- `coverage_planner/boustrophedon_planner.py` - area coverage planner and direct controller
- `coverage_planner/simple_mapper.py` - simple occupancy grid mapper
- `setup.py` - Python package metadata and console scripts
- `setup.cfg` - install path configuration for Python scripts
- `package.xml` - ROS package metadata and dependencies

## How to Run the System

There are three main stages:

1. simulation
2. mapping
3. coverage

You can also switch between two mapping modes:

- `slam` - use `slam_toolbox`
- `simple` - use the custom `simple_mapper`

---

## Stage 1: Start the Simulation

Open Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_description gazebo.launch.py
```

This will:

- open Gazebo
- spawn the robot into the arena
- start `robot_state_publisher`
- start the Gazebo-ROS bridge
- make `/scan`, `/odom`, `/joint_states`, `/imu`, `/tf`, and `/cmd_vel` available to ROS 2

---

## Stage 2: Build a Map

### Option A: SLAM Toolbox Mapping

Open Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_description slam.launch.py mapping_mode:=slam
```

This will start:

- `slam_toolbox`
- lifecycle manager for SLAM
- `teleop_twist_keyboard` in an `xterm` window
- RViz with the SLAM layout

Use the teleop window to drive the robot around the arena and let SLAM build the map.

### Option B: Simple Mapper

Open Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_description slam.launch.py mapping_mode:=simple
```

This will start:

- `simple_mapper` instead of `slam_toolbox`
- teleop in `xterm`
- RViz with the SLAM layout

This mode is simpler and maps the environment directly from `/scan` using a custom occupancy-grid update algorithm.

### What the Mapping Stage Produces

Both modes produce a `/map` topic and a live RViz map display.

- `slam` gives you a SLAM-generated map
- `simple` gives you a lightweight custom map

---

## Saving the Map

After exploring the arena, save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/robot_description/maps/arena_map --fmt png
```

If your workspace is elsewhere, replace the path accordingly.

The saved result is usually:

- `arena_map.yaml`
- `arena_map.png`

You can later load the YAML file in coverage mode.

---

## Stage 3: Area Coverage

Open a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_description coverage.launch.py map:=/absolute/path/to/arena_map.yaml
```

This launch will:

- include Nav2 bringup
- load the saved map
- start the coverage planner node
- open RViz with the coverage layout

### What Nav2 Is Doing Here

Nav2 is used as supporting infrastructure for:

- loading the saved map
- providing costmaps
- providing localization support
- handling navigation-related configuration

In this project, the coverage planner does **direct cmd_vel control**. The planner itself computes waypoints and publishes `/cmd_vel` directly.

### What the Coverage Planner Does

The planner node can:

- receive polygon points from RViz via `/clicked_point`
- finalize areas with `/coverage_command` messages
- validate areas against the map
- generate boustrophedon zigzag paths
- publish the path to RViz
- move the robot waypoint by waypoint using `/cmd_vel`
- cancel execution
- clear areas
- accept a single 2D goal pose from RViz

### Basic Coverage Workflow

1. In RViz, use **Publish Point** to click polygon vertices.
2. Send:

```bash
ros2 topic pub --once /coverage_command std_msgs/msg/String "{data: 'finish_area'}"
```

3. Add more areas if needed.
4. Start coverage:

```bash
ros2 topic pub --once /coverage_command std_msgs/msg/String "{data: 'start'}"
```

5. The planner publishes `/cmd_vel` directly and the robot follows the planned coverage path.

### Other Coverage Commands

Clear all areas:

```bash
ros2 topic pub --once /coverage_command std_msgs/msg/String "{data: 'clear'}"
```

Cancel the current coverage run:

```bash
ros2 topic pub --once /coverage_command std_msgs/msg/String "{data: 'cancel'}"
```

Mark the current polygon as an exclusion area:

```bash
ros2 topic pub --once /coverage_command std_msgs/msg/String "{data: 'exclude_area'}"
```

### Direct Goal Navigation in the Planner

The planner also listens to the RViz **2D Goal Pose** tool. If you click a goal pose, the planner will drive directly toward that point using its internal controller.

---

## How the Topics Work

### During Gazebo + SLAM

Typical data flow:

- Gazebo publishes sensor and robot state information
- `ros_gz_bridge` translates Gazebo messages to ROS 2 topics
- `slam_toolbox` or `simple_mapper` reads `/scan`
- the mapping node publishes `/map`
- RViz displays the map and TF frames
- teleop publishes `/cmd_vel`
- bridge forwards `/cmd_vel` to Gazebo

### During Coverage

Typical data flow:

- `coverage_planner` subscribes to `/map` and `/odom`
- user defines polygon areas in RViz
- planner generates coverage waypoints
- planner publishes `/cmd_vel`
- bridge forwards the command to Gazebo
- the robot moves in simulation

## Practical Notes

- `slam.launch.py` already starts teleop in an `xterm` window.
- `mapping_mode:=slam` and `mapping_mode:=simple` let you switch the mapping implementation.
- The planner in `boustrophedon_planner.py` does not rely on Nav2 for its final motion control.
- Static obstacles are filtered from the map during planning, but dynamic obstacle avoidance is not the same as a full Nav2 local planner.
- If you want more safety against moving obstacles, that is where Nav2 local planning would normally be used.

## Troubleshooting

### RViz opens but the robot does not move

Check that:

- Gazebo is running
- the workspace is sourced
- `/odom` exists
- `/cmd_vel` is being published
- the bridge is running

### SLAM does not publish a map

Check that:

- `/scan` is available
- the robot has been moved around
- `mapping_mode:=slam` is selected if you want SLAM Toolbox

### Coverage does not start

Check that:

- a valid map has been loaded
- at least one area has been finalized
- odometry has been received
- you have sent `start` on `/coverage_command`

### Teleop window does not appear

Install `xterm` and make sure the launch environment can open GUI windows.

### `map_saver_cli` fails

Make sure the `nav2_map_server` package is installed and the `/map` topic is actively publishing.

## Summary

This project provides a full ROS 2 simulation and coverage workflow:

1. run Gazebo
2. build a map with SLAM or the simple mapper
3. save the map
4. launch coverage mode
5. define areas in RViz
6. start coverage
7. let the custom boustrophedon planner drive the robot through `/cmd_vel`

If you want, the next step is to read the README again together with the source files so the documentation and code match line by line.
