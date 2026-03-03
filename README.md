# TurtleBot4STD
## TurtleBot4 Standard – Remote Node Command Library
A structured library of ROS 2 nodes and utility scripts for rapidly deploying motion and behavior commands to the TurtleBot4 Standard platform via SSH.
The goal of this repository is to enable:
One-line remote execution of robot behaviors
Rapid testing of mobility primitives
Modular expansion for multi-agent systems
Clean separation between high-level mission logic and low-level actuation
This repo supports development for distributed robotics systems (e.g., MARL, mission leader → agent control, search & navigation experiments).
## Project Goals
Build a reusable library of mobility primitives
Enable remote robot control via SSH
Standardize ROS 2 node structure
Reduce command friction during testing
Support single-agent and multi-agent deployments

## System Requirements
TurtleBot4 Standard
ROS 2 (Humble or Jazzy recommended)
Ubuntu 24.04
SSH access to robot
Same ROS_DOMAIN_ID across devices (if multi-robot)

## SSH Usage
SSH into robot:
ssh ubuntu@10.1.1.104
Then source ROS:
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_ws/install/setup.bash
Run a motion node:
ros2 run turtlebot4std rotate
## Example: One-Line Remote Execution
From your local machine:
ssh ubuntu@10.1.1.104 "source /opt/ros/jazzy/setup.bash && ros2 run turtlebot4std rotate"
This allows:
- Direct command execution
-  Quick behavior testing
- Multi-robot control scripts
## Basic Mobility Nodes
Rotate Node
Publishes to:
- /cmd_vel_unstamped
Uses:
- geometry_msgs/msg/Twist
- Example behavior:
- twist.angular.z = 0.5
- Forward Node
Publishes linear velocity:
twist.linear.x = 0.2
Stop Node
Publishes zero velocity to immediately halt motion.
## Multi-Robot Notes
If running multiple TurtleBots:
- Assign unique ROS_DOMAIN_ID
- Use namespaces if needed:
  - ros2 run turtlebot4std rotate --ros-args -r __ns:=/robot1
Check nodes:
- ros2 node list
- ros2 topic list
## Debugging
Check if /cmd_vel_unstamped is active:
- ros2 topic list
- Echo commands:
  - ros2 topic echo /cmd_vel
- Verify battery:
  - ros2 topic echo /battery_state
- If robot does not move:
  - Confirm /cmd_vel exists
  - Confirm robot is not docked
  - Confirm no conflicting nodes are publishing
## Extending the Library
To add a new behavior:
- Create a new node in /nodes
- Use rclpy
- Publish to /cmd_vel_unstamped
- Add entry point in setup.py
- Rebuild workspace:
  - colcon build
  - source install/setup.bash
  
## Integration With MARL / Mission Leader Architecture
This library is designed to:
- Receive directives from a central mission node
- Execute atomic motion primitives
- Remain stateless unless extended
- Operate independently if other agents go offline
This aligns with distributed autonomous systems and independent agent functionality requirements.