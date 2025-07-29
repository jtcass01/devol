# devol

**devol** is an example **ROS 2 Jazzy** and **Gazebo Harmonic** demonstration, focused on simulating a robotic pick-and-place task using a Universal Robot (UR) manipulator and a Robotiq gripper. The project integrates robot description, motion planning with MoveIt, Gazebo simulation, and task execution via ROS 2 nodes.
Packages include:
- devol_description: description of a UR manipulator with simple end effector and camera on wrist.
- diff_drive_description: description of a differential drive system.
- devol_drive_description: description of a Devol robot mounted ontop of a diff_drive system.
- dual_devol_description: description of two Devol Drive robots.

See docs/FinalProject_525_630_CASSADY_CHANDRA.pdf for a detailed report on the project.

[![Video Demonstration of Pick and Place](https://img.youtube.com/vi/oQB3xyw07hM/0.jpg)](https://www.youtube.com/watch?v=oQB3xyw07hM)
