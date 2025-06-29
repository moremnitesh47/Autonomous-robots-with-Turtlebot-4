# 🧭 Autonomous Exploration and Mapping using ROS 2

This project implements autonomous robot exploration and mapping using **ROS 2**, with a custom navigation stack (no `nav2`). It combines **frontier-based exploration**, **A\* path planning**, **map-based replanning**, and **dynamic obstacle avoidance** with real-time visualization in **RViz**.

---

## 🚀 Tasks Achieved

- ✅ Frontier-based exploration: find unexplored frontiers dynamically.
- ✅ Custom A\* path planning with clearance-based cost calculation.
- ✅ Replanning paths when the occupancy grid changes.
- ✅ Obstacle avoidance using LiDAR scan data.
- ✅ Real-time path visualization in RViz.
- ✅ Smooth robot motion using spline interpolation.

---

## 🧠 Core Concepts Used

### 🔹 Frontier-Based Exploration
- Frontiers are the borders between known and unknown areas.
- The robot continuously selects the **nearest reachable frontier centroid**.
- If a frontier becomes unreachable (due to obstacles), it is skipped.

### 🔹 A\* Path Planning (Custom Implementation)
- Operates on a grid-based occupancy map.
- Penalizes cells close to obstacles using a **clearance-based cost**.
- If the exact goal is not reachable, navigates to the **nearest reachable cell**.

### 🔹 Path Smoothing
- Applies **B-spline interpolation** using `scipy.interpolate.splprep/splev`.
- Increases smoothness of the path and avoids jerky movements.

### 🔹 Obstacle Detection & Escape
- Uses front-facing LiDAR data.
- If an obstacle is detected < 0.3m, the robot **rotates in place** to escape.

### 🔹 Dynamic Replanning
- Compares map updates to previous grid.
- If >5% of the map changes or if a timer triggers, **replans the path**.
- Cooldown avoids excessive CPU usage from frequent replanning.

---

## 🗂️ Package Structure

