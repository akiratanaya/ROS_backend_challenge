# Autonomous Mobile Robot System (ROS 2 Jazzy + Gazebo Harmonic)

## Project Overview
This project is an autonomous mobile robot system developed as part of the Robotics Case Study Submission. The system is built using **ROS 2 Jazzy Jalisco** and simulated within **Gazebo Harmonic**. 

The project addresses three main tasks:
1. **Robot and Simulation Setup:** A custom URDF robot model (inspired by Turtlebot3 dimensions) spawned inside a custom 10x10 meter Gazebo arena with 3 static obstacle cylinders.
2. **SLAM and Map Building:** Mapping the arena using `slam_toolbox` and saving it as a 2D occupancy grid, optimized for ROS 2 Jazzy containerized environments.
3. **Area Coverage System:** A Boustrophedon path planner that takes arbitrary user-defined polygon areas via RViz2, computes a zigzag coverage path, and navigates the robot through the path autonomously using a custom direct P-controller (bypassing Nav2 lifecycle instabilities).

---

## The System Architecture
The system follows a modular architecture divided into two primary packages:

1. **`robot_description`**: 
   - Contains the custom URDF/Xacro model, physical properties, and sensor plugin definitions (GPU LiDAR, IMU, Diff Drive).
   - Contains the `arena.sdf` world definition for Gazebo Harmonic.
   - Handles the `ros_gz_bridge` configurations to sync simulation clocks, TF trees, and sensor data to the ROS 2 ecosystem.
   - Includes custom launch files for Gazebo, SLAM (using `nav2_lifecycle_manager`), and RViz visualizations.

2. **`coverage_planner`**:
   - **`boustrophedon_planner`**: The core logic node. It listens for `PointStamped` inputs from RViz2, forms a polygon, and calculates optimal sweeping lines. It features a robust **direct control system** that calculates distance/heading errors via `/odom` and publishes directly to `/cmd_vel`. This architectural decision was made to ensure 100% reliability against ROS 2 Jazzy's `bt_navigator` action server timeout bugs.

---

## How to Setup

### Prerequisites
- **Ubuntu 24.04** (or Ubuntu 22.04 inside Distrobox/Docker)
- **ROS 2 Jazzy Jalisco**
- **Gazebo Harmonic**
- Basic ROS 2 build tools (`colcon`, `rosdep`)

### Installation Steps
1. Clone this repository into your ROS 2 workspace `src` directory:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone <your-github-repo-url>
   ```

2. Install system dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep update
   rosdep install --from-paths src -y --ignore-src
   ```

3. Build the packages:
   ```bash
   colcon build --symlink-install
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## How to Use & Configuration Guide

### Task 1 & 2: Simulation and SLAM Mapping
To start mapping the 10x10 arena:

1. **Launch Gazebo Simulation:**
   Open a terminal and run:
   ```bash
   source install/setup.bash
   ros2 launch robot_description gazebo.launch.py
   ```

2. **Launch SLAM & RViz:**
   Open a second terminal and run:
   ```bash
   source install/setup.bash
   ros2 launch robot_description slam.launch.py
   ```

3. **Teleoperate the Robot:**
   Open a third terminal and run:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Drive the robot around the arena using `i, j, k, l, ,` keys to map the environment.

4. **Save the Map:**
   Once the arena is fully explored, save the map in `.png` format (to prevent ImageMagick segmentation faults in Jazzy):
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/robot_description/maps/arena_map --fmt png
   ```

### Task 3: Area Coverage (Boustrophedon)
*This task utilizes our custom launch file that brings up Nav2 Costmaps and the Boustrophedon Planner.*

1. **Launch Coverage System:**
   Start the coverage environment with the saved map:
   ```bash
   ros2 launch robot_description coverage.launch.py map:=/home/akiratanaya/robotics/seleksi_itb_delabo/ros_backend/ROS_backend_challenge/src/robot_description/maps/arena_map.yaml
   ```

2. **Initialize Robot Pose:**
   - In RViz, use the **2D Pose Estimate** tool to set the robot's initial starting location (crucial for AMCL localization and Costmap bringup).

3. **Feature A: Direct Goal Navigation:**
   - You can use the **2D Goal Pose** tool in RViz to click anywhere on the map. The custom planner will seamlessly navigate the robot directly to that point.

4. **Feature B: Boustrophedon Area Coverage:**
   - Use the **Publish Point** tool in RViz to click 4 times on the map, defining a polygonal coverage zone.
   - In a separate terminal, finalize the area:
     ```bash
     ros2 topic pub --once /coverage_command std_msgs/msg/String "{data: 'finish_area'}"
     ```
   - Start the autonomous coverage sweep:
     ```bash
     ros2 topic pub --once /coverage_command std_msgs/msg/String "{data: 'start'}"
     ```
   - The robot will execute a highly accurate zigzag path across the defined polygon using its internal P-controller.

---