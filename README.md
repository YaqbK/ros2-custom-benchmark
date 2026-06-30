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

```bash
# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# Navigate to your workspace root (e.g., ~/custom_benchmark_ws)
# Build the workspace
colcon build

# Source the local setup
source install/setup.bash
```

## 🖥️ Usage
1. Environment Generation
Generate a random scene with obstacles and a set of planning queries.

```bash
ros2 launch ur5_dataset_generator dataset.launch.py
```

2. Dataset Verification
Visually verify the randomized obstacle density and robot configurations in RViz to ensure they are physically sensible.

```
ros2 launch ur5_dataset_generator visualize_dataset.launch.py
```

3. Benchmark Execution
Launch the main custom benchmarking node. This will iteratively test all configured planning pipelines against the generated queries.

```
ros2 launch my_custom_benchmarks run_benchmark.launch.py
```

4. Data Processing
Convert the raw MoveIt log files into an SQLite database for statistical analysis (e.g., via Planner Arena).

```
ros2 run moveit_ros_benchmarks moveit_benchmark_statistics.py [path_of_log_file] [path_of_saved_db_file]
```

## 🔮 Future Work
* Refinement of the random generator to prevent complete workspace blockage.
* Parameter tuning for highly constrained 6D C-Space bottlenecks.
* Full compilation and hardware verification of the GPU-accelerated cuRobo planning interface.

## 👨‍💻 Author

Jakub Krusicki
  
Automatic Control and Robotics, Poznań University of Technology.
