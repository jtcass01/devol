# devol

**devol** is an example **ROS 2 Jazzy** and **Gazebo Harmonic** demonstration, focused on simulating a robotic pick-and-place task using a Universal Robot (UR) manipulator and a Robotiq gripper. The project integrates robot description, motion planning with MoveIt, Gazebo simulation, and task execution via ROS 2 nodes.

See docs/FinalProject_525_630_CASSADY_CHANDRA.pdf for a detailed report on the project.

[![Video Demonstration of Pick and Place](https://img.youtube.com/vi/oQB3xyw07hM/0.jpg)](https://www.youtube.com/watch?v=oQB3xyw07hM)


---

## 📦 Packages Overview

### devol_application

Contains the logic for executing a pick-and-place task.

- `pick_and_place.cpp`: A ROS 2 node that manages the behavior of the robot to perform pick-and-place operations, using MoveIt for motion planning and controlling the Robotiq gripper.

### devol_bringup

Launch package for initializing the full simulation environment.

- `devol_sim.launch.py`: Launches the complete simulation, including Gazebo, the robot model, and all necessary nodes.

### devol_description

Defines the robot’s physical and visual properties.

- Xacro files for the UR manipulator and Robotiq gripper
- Combined robot description for simulation and control
- Integrates with `ros2_control` for hardware simulation and control interfaces

### devol_gazebo

Configures the Gazebo simulation environment.

- World files in SDF format (e.g., `factory_world.sdf`)
- Sources world objects from [Fuel](https://app.gazebosim.org/dashboard)

### devol_moveit_config

Configuration for MoveIt 2 to plan and execute motions for the devol robot.

- Motion planning pipelines
- Kinematics and controller configuration
- RViz motion planning plugin setup


