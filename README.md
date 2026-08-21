# ROS 2 Inspection Robot

A simulated autonomous inspection robot built with **ROS 2 Humble**, **Gazebo Classic**, and **RViz2**.

The project demonstrates the core components of a mobile inspection robot:

* Differential-drive movement
* Wheel odometry
* TF transforms
* LiDAR obstacle detection
* Camera sensing
* Reactive obstacle avoidance
* Goal-based navigation
* Gazebo simulation
* RViz visualization

## Project Architecture

```text
                         ┌──────────────────┐
                         │      Gazebo      │
                         │                  │
                         │ Robot + World    │
                         └────────┬─────────┘
                                  │
                 ┌────────────────┼────────────────┐
                 │                │                │
              /odom            /scan        /camera/image_raw
                 │                │                │
                 ▼                ▼                ▼
          Differential      LiDAR sensor       Camera
             Drive               │
                 │                │
                 │                ▼
                 │       Obstacle Avoidance
                 │                │
                 │                │ /cmd_vel
                 │                │
                 └────────┬───────┘
                          ▼
                       Robot

                     Navigation
                          │
                       /scan
                       /odom
                          │
                          ▼
                  maze_navigator
                          │
                       /cmd_vel
                          │
                          ▼
                   Differential Drive
```

## Package Structure

```text
ros2_ws/
└── src/
    ├── inspection_robot/
    │   ├── inspection_robot/
    │   │   ├── camera_node.py
    │   │   ├── drive_node.py
    │   │   ├── lidar_node.py
    │   │   └── obstacle_avoidance.py
    │   │
    │   ├── launch/
    │   │   ├── inspection.launch.py
    │   │   └── display.launch.py
    │   │
    │   ├── urdf/
    │   │   └── inspection_robot.urdf.xacro
    │   │
    │   ├── worlds/
    │   │   └── obstacle_world.world
    │   │
    │   ├── rviz/
    │   │   └── display.rviz
    │   │
    │   ├── resource/
    │   │   └── inspection_robot
    │   │
    │   ├── package.xml
    │   └── setup.py
    │
    └── maze_navigator/
        ├── maze_navigator/
        │   └── maze_navigator.py
        ├── package.xml
        └── setup.py
```

## Sensors

### LiDAR

The robot uses a 360-degree simulated LiDAR.

Topic:

```text
/scan
```

Message type:

```text
sensor_msgs/msg/LaserScan
```

The LiDAR is used by both RViz and the navigation/obstacle-avoidance systems.

### Camera

The robot has a simulated inspection camera.

Main image topic:

```text
/camera/image_raw
```

Camera information:

```text
/camera/camera_info
```

The camera provides visual inspection capability inside the simulated environment.

## Robot Motion

The differential-drive controller publishes:

```text
/odom
```

and consumes:

```text
/cmd_vel
```

The robot's main TF structure is:

```text
odom
 └── base_link
      ├── left_wheel_link
      ├── right_wheel_link
      ├── lidar_link
      └── camera_link
```

## Launching the Simulation

Build the workspace:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Launch the simulation:

```bash
ros2 launch inspection_robot inspection.launch.py
```

This starts:

* Gazebo
* The obstacle world
* Robot spawning
* Robot State Publisher
* RViz

Navigation and obstacle-avoidance nodes are intentionally started separately.

## Obstacle Avoidance

Start the reactive obstacle-avoidance controller:

```bash
ros2 run inspection_robot avoid
```

The controller reads:

```text
/scan
```

and publishes:

```text
/cmd_vel
```

It evaluates the environment using front, left, and right LiDAR sectors.

## Goal Navigation

The autonomous navigation node is:

```bash
ros2 run maze_navigator maze_navigator
```

The navigator uses:

```text
/odom
/scan
```

and publishes:

```text
/cmd_vel
```

The navigation system combines the robot's position and heading with LiDAR obstacle information to move toward the configured goal while avoiding obstacles.

## Useful ROS 2 Commands

Check active nodes:

```bash
ros2 node list
```

Check available topics:

```bash
ros2 topic list
```

Check odometry:

```bash
ros2 topic echo /odom
```

Check LiDAR:

```bash
ros2 topic echo /scan
```

Check command velocity:

```bash
ros2 topic echo /cmd_vel
```

Check LiDAR publisher:

```bash
ros2 topic info /scan -v
```

Check odometry frequency:

```bash
ros2 topic hz /odom
```

Check TF:

```bash
ros2 run tf2_tools view_frames
```

## Development Status

### Completed

* [x] ROS 2 workspace
* [x] Inspection robot URDF/Xacro
* [x] Differential-drive controller
* [x] Wheel joints
* [x] Odometry
* [x] TF publishing
* [x] Gazebo simulation
* [x] Custom obstacle world
* [x] 360-degree LiDAR
* [x] Camera sensor
* [x] RViz visualization
* [x] Reactive obstacle avoidance
* [x] Separate autonomous navigation executable
* [x] ROS 2 launch system
* [x] Git/GitHub repository

### In Progress

* [ ] Final navigation tuning
* [ ] Faster autonomous navigation
* [ ] Improved obstacle recovery behavior
* [ ] Final demonstration/testing

## Technologies

* ROS 2 Humble
* Python
* Gazebo Classic
* RViz2
* URDF/Xacro
* ROS 2 `rclpy`
* Git/GitHub

## Repository

This project is part of a collection of ROS 2 robotics experiments focused on simulation, sensing, control, and autonomous navigation.
