"""Local motion planning stack: the PID path follower plus the selected planner.

  ros2 launch devol_local_planner local_planner.launch.py x:=0.0 y:=0.0 yaw:=0.0            # RRT* (default)
  ros2 launch devol_local_planner local_planner.launch.py planner:=a_star x:=0.0 y:=0.0 yaw:=0.0

Needs a running sim (devol_sim) for the map, goals and TF. devol_sim's motion_planner_sim.launch.py
includes this file and passes the robot's spawn pose from the world's poses.csv.
"""

from math import pi

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    namespace = arg('namespace')
    planner = arg('planner')

    pid_controller = Node(
        package='devol_local_planner',
        executable='diffdrive_pid',
        name='diffdrive_pid',
        output='screen',
        parameters=[
            {
                'kp': 2.3,
                'ki': 0.01,
                'kd': 0.5,
                'lookahead': 0.25,
                'publish_rate': 30.0,
                'max_linear_vel': 1.0,
                'max_angular_vel': pi,
                'x': float(arg('x')),
                'y': float(arg('y')),
                'yaw': float(arg('yaw')),
                'namespace': namespace,
            }
        ],
    )

    map_padder = Node(
        package='devol_local_planner',
        executable='map_padder',
        name='map_padder',
        output='screen',
        namespace=namespace,
        parameters=[
            {
                'namespace': namespace,
                'robot_width': 1.0,  # In m
            }
        ],
    )

    agent_motion_planner = Node(
        package='devol_local_planner',
        executable='agent_motion_planner',
        name='agent_motion_planner',
        output='screen',
        parameters=[
            {
                'publish_rate': 10.0,
                'namespace': namespace,
                'lookahead': 0.25,
                'goal_tolerance': 0.1,
                'intermediate_goal_tolerance': 0.4,
            }
        ],
        arguments=['--ros-args', '--log-level', 'agent_motion_planner:=info'],
    )

    rrt_motion_planner = Node(
        package='devol_local_planner',
        executable='rrt_motion_planner',
        name='rrt_motion_planner',
        output='screen',
        parameters=[
            {
                'publish_rate': 10.0,
                'namespace': namespace,
                'lookahead': 0.25,
                'goal_tolerance': 0.1,
                'intermediate_goal_tolerance': 0.4,
                'map_source': 'octomap_3d',
                'algorithm': planner if planner != 'a_star' else 'rrt_star',
            }
        ],
    )

    if planner == 'a_star':
        planner_nodes = [map_padder, agent_motion_planner]
    else:
        planner_nodes = [rrt_motion_planner]

    return [pid_controller, *planner_nodes]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace', default_value='/devol_drive', description='Namespace for topics'
            ),
            DeclareLaunchArgument(
                'planner',
                default_value='rrt_star',
                choices=['a_star', 'rrt', 'rrt_star'],
                description='Local planner: a_star (padded C-space map) or rrt / rrt_star (robot footprint on the octomap projected_map)',
            ),
            DeclareLaunchArgument('x', default_value='0.0', description='Robot start x (m)'),
            DeclareLaunchArgument('y', default_value='0.0', description='Robot start y (m)'),
            DeclareLaunchArgument('yaw', default_value='0.0', description='Robot start yaw (rad)'),
            OpaqueFunction(function=launch_setup),
        ]
    )
