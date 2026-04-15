# UR5e MoveIt 2 Benchmark (ROS 2 Humble)

This repository contains a custom benchmarking infrastructure for motion planning on the Universal Robots UR5e, specifically developed for ROS 2 Humble.

## Overview
The standard MoveIt 2 benchmarking tutorials and scripts often encounter critical parameter-type mismatches in the Humble distribution (e.g., conflicts between `string` and `string_array` for planner names). 

This project solves these issues by implementing a **custom C++ Benchmark Executor** that:
- Bypasses the broken YAML parameter parser.
- Manually injects planning pipelines and algorithms into memory.
- Overrides the `BenchmarkExecutor` class to handle database-less queries.

## Features
- **Robot:** UR5e (using standard `ur_moveit_config`).
- **Planners:** RRTConnect, PRM, and RRTstar.
- **Environment:** Customizable C++ planning scene (ready for obstacle integration).
- **Output:** Generates standard MoveIt benchmark log files for statistical analysis.

## Prerequisites
- ROS 2 Humble
- MoveIt 2
- [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

## Installation & Usage
1. Clone to your workspace `src` folder.
2. Build the package:
   colcon build --packages-select my_custom_benchmarks
   source install/setup.bash
