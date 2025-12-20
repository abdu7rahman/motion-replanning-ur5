# Dynamic Pick and Place with Reactive Motion Replanning

Reactive motion replanning system for UR5 manipulator that exploits kinematic redundancy to handle dynamic obstacles during pick-and-place operations.

**ME5250 - Robot Mechanics and Control | Northeastern University**

Mohammed Abdul Rahman

## Demo Video

[![Demo Video](https://img.youtube.com/vi/wx9h6yUB3TI/0.jpg)](https://www.youtube.com/watch?v=wx9h6yUB3TI )

## Overview

This project implements Strategy 1 reactive replanning: when obstacles appear mid-execution, the system detects the collision, aborts the current trajectory, and evaluates alternative joint configurations to find collision-free paths without full trajectory regeneration.

A 6-DOF manipulator reaching a 6-DOF Cartesian pose admits multiple valid joint configurations due to kinematic redundancy (elbow up/down, shoulder left/right, wrist flips). By pre-computing 15-20 IK solutions per target pose, the system can rapidly switch to valid alternatives when obstacles block the primary path.

## Features

- Reactive replanning using kinematic redundancy
- Mid-execution obstacle detection and trajectory abortion
- 15-20 distinct IK solutions computed per target pose
- 1-3 second replanning latency
- Robotiq 2F-85 gripper integration
- Dynamic obstacle insertion via command line
- Point cloud perception pipeline (work in progress)

## System Stack

- ROS2 Jazzy
- MoveIt2
- Gazebo Harmonic
- Ubuntu 24.04
- UR5 + Robotiq 2F-85

## Installation
```bash
git clone https://github.com/abdu7rahman/motion-replanning-ur5.git
cd ur_ws/
colcon build
source install/setup.bash
```

## Usage

### Launch simulation
```bash
ros2 launch ur5_moveit simulated_robot.launch.py
```

### Run pick and place
```bash
# Single cube
python3 src/ur5_moveit/ur5_moveit/pick_and_place.py

# All 3 cubes
python3 src/ur5_moveit/ur5_moveit/pick_and_place_advanced.py
```

### Insert obstacle (in separate terminal)
```bash
# Add obstacle
python3 src/ur5_moveit/ur5_moveit/insert_obstacle.py --x 0.3 --y -0.2 --z 0.5 --radius 0.04 --height 0.25

# Remove obstacle
python3 src/ur5_moveit/ur5_moveit/insert_obstacle.py --name obstacle --remove
```

## How It Works

1. Compute multiple IK solutions for target pose using randomized seeds
2. Begin trajectory execution toward first solution
3. Monitor `/collision_object` topic for obstacles
4. On obstacle detection:
   - Abort current trajectory
   - Iterate through remaining IK solutions
   - Plan and execute first collision-free configuration
5. Continue pick-and-place operation

## Results

| Metric | Value |
|--------|-------|
| IK solutions per pose | 15-20 |
| Replanning latency | 1-3 seconds |
| IK discovery rate | 15-20% from 100 attempts |

## Project Structure
```
├── ur5_moveit/
│   ├── pick_and_place.py          # Single cube demo
│   ├── pick_and_place_advanced.py # All 3 cubes
│   ├── insert_obstacle.py         # Dynamic obstacle insertion
│   └── pointcloud_to_planning_scene.py  # Perception (WIP)
├── worlds/
│   └── pick_place.world           # Gazebo world with tables and cubes
└── config/
    └── ...                        # MoveIt2 configs
```

## Acknowledgments

This project builds upon the [UR5 ROS2 Workspace](https://github.com/nithishreddy1101/ur5_ws) by Nithish Reddy, which provided pre-configured Gazebo Harmonic integration with the Robotiq 2F-85 gripper.

## Future Work

- Strategy 2: Predictive replanning with time-to-collision estimation
- Strategy 3: CHOMP/TrajOpt-style local trajectory optimization
- Complete point cloud perception for automatic obstacle detection
- Quantitative evaluation with formal trials
- Physical robot deployment
- Move to a UR10e

## License

MIT

## Author

Mohammed Abdul Rahman  
MS Robotics, Northeastern University  
mohammedabdulr.1@northeastern.edu
