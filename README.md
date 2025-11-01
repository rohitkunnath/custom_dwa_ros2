# custom_dwa_ros2

# Custom DWA Local Planner for TurtleBot3 (ROS2 Humble)

This project implements a custom DWA (Dynamic Window Approach) local planner for TurtleBot3 using ROS2 Humble in a Gazebo simulation.

## Quick Start

1. Clone this repo inside your ROS2 workspace `src` folder and build: cd ~/dwa_ws

```bash
cd ~/dwa_ws
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch my_worlds my_custom_launch.py
```

2. Use RViz to send a goal using Nav2 goal and watch the robot navigate using the custom DWA.

## Requirements

- ROS2 Humble
- Gazebo 11+
- TurtleBot3 packages

