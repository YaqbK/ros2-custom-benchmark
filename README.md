# ROS 2 Custom Benchmarking Environment for UR5e

A unified, automated pipeline for generating, verifying, and benchmarking motion planning algorithms for the UR5e manipulator. Built with ROS 2 Humble and MoveIt 2, this project provides a standardized framework to compare sampling-based planners (OMPL) against gradient-based trajectory optimization techniques (CHOMP) under identical environmental constraints.

## 🚀 Features

* **Unified C++ API:** A standardized execution wrapper that bridges the gap between fragmented planner interfaces (OMPL, CHOMP).
* **Automated Dataset Generation:** Randomly instantiates geometric obstacles (boxes, cylinders, spheres) and mathematically samples valid, collision-free start and goal joint configurations.
* **Interactive Dataset Visualizer:** A custom command-line interface (CLI) node linked to RViz for manual, deterministic inspection of generated query pairs before running heavy benchmarks.
* **Data Separation:** Clean separation of physical environments (`obstacles.yaml`) and joint configurations (`queries.yaml`) to ensure repeatable testing.
* **cuRobo Ready:** Modular architecture including prepared Python boilerplate for future GPU-accelerated planner integration (NVIDIA cuRobo).

## 🛠️ Prerequisites

* **OS:** Ubuntu 22.04
* **Middleware:** ROS 2 Humble
* **Framework:** MoveIt 2
* **Dependencies:** `moveit_ros_benchmarks`, Flexible Collision Library (FCL)

## 📦 Installation & Build

Clone this repository into the `src` directory of your ROS 2 workspace, resolve dependencies, and build using `colcon`.

# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# Navigate to your workspace root (e.g., ~/custom_benchmark_ws)
# Build the workspace
colcon build

# Source the local setup
source install/setup.bash
