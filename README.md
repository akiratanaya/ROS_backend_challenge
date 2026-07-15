# Autonomous Mobile Robot System (ROS 2 Jazzy + Gazebo Harmonic)

## 📌 Project Overview
This project is an autonomous mobile robot system developed as part of the Robotics Case Study Submission. The system is built using **ROS 2 Jazzy Jalisco** and simulated within **Gazebo Harmonic**. 

The project addresses three main tasks:
1. **Robot and Simulation Setup:** A custom URDF robot model (inspired by Turtlebot3 dimensions) spawned inside a custom 10x10 meter Gazebo arena with 3 static obstacle cylinders.
2. **SLAM and Map Building:** A custom 2D Occupancy Grid Mapper that robustly generates arena maps from LiDAR and Odometry data via teleoperation.
3. **Area Coverage System:** A Boustrophedon path planner that takes arbitrary user-defined polygon areas via RViz2, computes a zigzag coverage path while excluding obstacles, and navigates the robot through the path using Nav2 Action Clients.

---

## 🏗️ The System Architecture
The system follows a modular architecture divided into two primary packages:

1. **`robot_description`**: 
   - Contains the custom URDF/Xacro model, physical properties, and sensor plugin definitions (GPU LiDAR, IMU, Diff Drive).
   - Contains the `arena.sdf` world definition for Gazebo Harmonic.
   - Handles the `ros_gz_bridge` configurations to sync simulation clocks, TF trees, and sensor data to the ROS 2 ecosystem.
   - Includes custom launch files for Gazebo and RViz visualizations.

2. **`coverage_planner`**:
   - **`simple_mapper`**: A custom python-based node that subscribes to `/scan` and TF to mathematically build a 2D occupancy grid (`/map`). This bypasses limitations found with `slam_toolbox` in containerized (Distrobox) environments.
   - **`boustrophedon_planner`**: The core logic node. It listens for `PointStamped` inputs from RViz2, forms a polygon, calculates optimal sweeping lines, filters out known obstacles via the global costmap, and issues navigation commands to `nav2_bringup`.

---

## ⚙️ How to Setup

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

## 🚀 How to Use & Configuration Guide

### Task 1 & 2: Simulation and SLAM Mapping
To start mapping the 10x10 arena:

1. **Launch Gazebo Simulation:**
   Open a terminal and run:
   ```bash
   source install/setup.bash
   ros2 launch robot_description gazebo.launch.py
   ```
   *Note for Container/Distrobox users:* If you experience LiDAR rendering issues due to lack of direct GPU access, the launch file uses software rendering fallback (`LIBGL_ALWAYS_SOFTWARE`) with Ogre1, ensuring the CPU handles raycasting properly.

2. **Launch the Mapper & RViz:**
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
   Drive the robot around the arena using `i, j, k, l, ,` keys. The custom mapper will instantly draw the walls in RViz2 based on a 12.0m max laser range.

4. **Save the Map:**
   Once the arena is fully explored, save the map for the coverage task:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/robot_description/maps/arena_map
   ```

### Task 3: Area Coverage (Boustrophedon)
*Assuming Nav2 is configured to load the previously saved map.*

1. **Launch Nav2 and Planner:**
   Start the standard `nav2_bringup` with your saved map, and then run the coverage node:
   ```bash
   ros2 run coverage_planner boustrophedon_planner
   ```

2. **Define Coverage Area:**
   - In RViz2, use the **Publish Point** tool on the top toolbar.
   - Click at least 3 times on the map to define the vertices of your arbitrary polygonal coverage zone.
   - The planner will validate the area. If it overlaps with obstacles (costmap threshold), those specific cells are excluded from the coverage path.

3. **Execution:**
   - The node will compute a zigzag (boustrophedon) trajectory.
   - It sends `NavigateThroughPoses` action goals to Nav2.
   - The robot will autonomously sweep the defined area while avoiding the 3 static cylinders.

---

## 📝 Limitations & Future Improvements
- **Mapping Robustness:** The current custom mapper assumes perfect odometry and does not perform loop closure or advanced scan matching like Cartographer/SLAM Toolbox. It is highly effective for simulation but would drift in real-world scenarios with wheel slip.
- **Coverage Sorting:** The current Boustrophedon planner executes sub-regions sequentially. Implementing an optimizer (like TSP or Nearest Neighbor) to sort disjoint areas would make the cleaning/coverage path significantly faster.
- **Dynamic Obstacles:** The planner currently relies on the static costmap. Future iterations should incorporate the local costmap to dynamically reroute mid-sweep if a moving obstacle appears.
