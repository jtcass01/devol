# devol

**devol** is an example **ROS 2 Jazzy** and **Gazebo Harmonic** demonstration, focused on simulating a robotic pick-and-place task using a Universal Robot (UR) manipulator and a Robotiq gripper. The project integrates robot description, motion planning with MoveIt, Gazebo simulation, and task execution via ROS 2 nodes.
Packages include:
- devol_description: description of a UR manipulator with simple end effector and camera on wrist.
- diff_drive_description: description of a differential drive system.
- devol_drive_description: description of a Devol robot mounted ontop of a diff_drive system. (in-progress)
- diff_drive_sim: A collection of nodes for controlling a diff_drive robot.

Code Tested Directly on Ubuntu 24.04. Docker container tested on RHEL8 OS.

## 1.0 Directions for Running Diff Drive Simulation
If you are managing the packages on your own without the need for docker, you can skip instructions 1.2 and 1.3.

### 1.1 Clone Git Repository
```bash
git clone https://github.com/jtcass01/devol.git
```

### 1.2 Build and Start Docker container
If you don't have docker compose, you can install it here: https://docs.docker.com/compose/install/.
Once it is installed run the following command to build the docker container from the root directory of the repository.

```bash
. bin/docker/build_sim_container.sh && . bin/docker/start.sh
```

### 1.3 Enter the docker Container
```bash
. bin/docker/interactive.sh
```

### 1.4 Build the packages
```bash
. bin/build.sh
```

### 1.5 Source Devol packages
```bash
. install/setup.bash
```

### 1.6 Run full simulation. -- You should see the robot plan and execute pathing to three goal points.
```bash
. bin/run.sh
```