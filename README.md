# ROS2 Agribot Workspace

This is a ROS2 workspace containing the Agribot project with Jazzy compatibility.

## Packages

- **articubot_one**: Main robot package with launch files, configurations, and URDF descriptions
- **serial_motor_demo**: Motor control demonstration package
- **serial_motor_demo_msgs**: Custom message definitions for motor control
- **mpu6050_driver**: IMU sensor driver package

## Features

- Gazebo Harmonic simulation support
- RViz visualization
- Navigation stack integration
- Joystick control
- Camera and LiDAR sensors
- Differential drive control

## Build Instructions

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

Launch simulation:
```bash
ros2 launch articubot_one launch_sim.launch.py
```

Launch real robot:
```bash
ros2 launch articubot_one launch_robot.launch.py
```

## Requirements

- ROS2 Jazzy
- Gazebo Harmonic
- ros2_control
- navigation2