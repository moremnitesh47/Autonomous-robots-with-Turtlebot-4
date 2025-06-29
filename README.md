# 🗭 Autonomous Exploration and Mapping using ROS 2 with Turtlebot4

---
Authors- Nitesh morem.

 Ostbayerische Technische Hochschule Amberg-Weiden.
 
 Department of Electrical Engineering, Media and Computer Science.
 
 Supervisor - Prof. Dr. -Ing. Thomas Nierhoff


---


This repository implements autonomous exploration and mapping using **ROS 2**, a simulated TurtleBot4 robot, and custom navigation logic. The solution is built completely **without Nav2**, using your own:

- A\* Path Planning
- Frontier Detection and Clustering
- Exploration Logic
- Dynamic Replanning
- Obstacle Avoidance
- RViz visualization

---

## 🚀 Tasks Achieved

- ✅ **Mapping Node**: Builds an occupancy grid map from LiDAR data.
- ✅ **Frontier Detection**: Detects unexplored boundaries.
- ✅ **Exploration Node**: Chooses reachable frontiers and navigates using custom A\* algorithm.
- ✅ **Path Planning**: Computes a clearance-aware path and publishes it.
- ✅ **Obstacle Avoidance**: Detects obstacles dynamically using LiDAR and performs escape maneuvers.
- ✅ **Visualization**: Frontiers and paths visualized in RViz.

---

## 🧠 Core Concepts and Algorithms

### 🔹 Frontier-Based Exploration
- Frontiers are the borders between known and unknown areas.
- The robot continuously selects the **nearest reachable frontier centroid**.
- If a frontier becomes unreachable (due to obstacles), it is skipped.

### 🔹 A\* Path Planning (Custom Implementation)
- Operates on a grid-based occupancy map.
- Penalizes cells close to obstacles using a **clearance-based cost**.
- If the exact goal is not reachable, navigates to the **nearest reachable cell**.

```python
clearance_penalty = 10 if clearance == 0 else 2.0 / clearance
```

### 🔹 Path Smoothing




Uses B-spline interpolation from `scipy`:
```python
x_coords, y_coords = zip(*path)
tck, _ = splprep([x_coords, y_coords], s=2)
u_new = np.linspace(0, 1, num=len(path) * 5)
smooth_x, smooth_y = splev(u_new, tck)
```
Filtered with obstacle-aware validation using:
```python
self.is_valid_cell(x, y, inflation_radius=0.5)
```

### 🔹 Obstacle Detection & Escape
- Uses front-facing LiDAR data.
- If an obstacle is detected < 0.3m, the robot **rotates in place** to escape.

### 🔹 Dynamic Replanning
- Compares map updates to previous grid.
- If >5% of the map changes or if a timer triggers, **replans the path**.
- Cooldown avoids excessive CPU usage from frequent replanning.

---

## 🗂️ Package Structure
```
ros2_ws5/
├── src/
    └── lidar_mapping/
        ├── launch/
        │   └── mapping_rviz2.launch.py
        ├── lidar_mapping/
        │   ├── exploration_node.py
        │   └── mapping_node.py
        ├── package.xml
        └── setup.py
```

---

## 🔧 Build and Run

### Prerequisites
- ROS 2 Humble
- Python3, `numpy`, `scipy`, `rclpy`

### Build Workspace
```bash
cd ~/ros2_ws5
colcon build
source install/setup.bash
```

### Launch
```bash
ros2 launch lidar_mapping mapping_rviz2.launch.py
```

This will start:
- Mapping node
- Exploration node
- RViz with pre-configured displays

---

## 📊 Visualization in RViz
- Frame: `map`
- Topics:
  - `/map` (`nav_msgs/OccupancyGrid`)
  - `/planned_path` (`nav_msgs/Path`)
  - `/frontier_centroids` (`visualization_msgs/MarkerArray`)
  - `/scan` (`sensor_msgs/LaserScan`)
  - `/odom` and `/tf`

---

## 🔀 Topics Summary

| Topic | Message Type | Description |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | 2D map used for planning |
| `/scan` | `sensor_msgs/LaserScan` | Obstacle detection |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | Robot pose |
| `/planned_path` | `nav_msgs/Path` | Path for visualization |
| `/frontier_centroids` | `visualization_msgs/MarkerArray` | Frontier visualization |

---

## 🌟 Key Highlights
- No Nav2 dependency 
- Lightweight custom exploration logic
- Robust to blocked frontiers
- Works in simulation or with real robot given odom, scan, and map inputs

---

## 🙌 Acknowledgments
Thanks to the open-source community, ROS 2, and the TurtleBot simulation environments.
