import os
import csv

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_pkg_share = get_package_share_directory('diff_drive_sim')
    gz_pkg_share = get_package_share_directory('devol_gazebo')

    # Declare maze selection argument
    maze_arg = DeclareLaunchArgument(
        'maze',
        default_value='Maze_ng',
        description='Maze to load: basic_maze or Maze_ng'
    )
    
    def launch_setup(context):
        maze_folder = context.perform_substitution(LaunchConfiguration('maze'))
        world_file = os.path.join(gz_pkg_share, 'worlds', maze_folder, 'maze_world.sdf')

        # Poses CSV file
        poses_file = os.path.join(gz_pkg_share, 'worlds', maze_folder, 'poses.csv')

        # Read poses from CSV
        poses = {}
        with open(poses_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                poses[row['name']] = {
                    'x': row['x'],
                    'y': row['y'],
                    'z': row['z'],
                    'yaw': row['yaw']
                }

        # Spawn the vehicle model in Gazebo
        robot_pose = poses['robot']

        # Launch Gazebo with the world
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                )
            ]),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        )

        # Include spawn entities launch file (robot, goals, map, static transforms)
        spawn_entities = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg_share, 'launch', 'spawn_entities.launch.py')
            ),
            launch_arguments={'maze': maze_folder}.items()
        )

        # Bridge, PID, RViz
        bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg_share, 'launch', 'ros_gz_bridge.launch.py')
            )
        )

        pid_controller = Node(
            package='diff_drive_sim',
            executable='diffdrive_pid',
            name='diffdrive_pid',
            output='screen',
            parameters=[
                {'kp': 0.8},
                {'kd': 0.2},
                {'ki': 0.0},
                {'lookahead': 0.1},
                {'publish_rate': 30.0},
                {'max_linear_vel': 0.75},
                {'max_angular_vel': 1.0},
                {'x': float(robot_pose['x'])},
                {'y': float(robot_pose['y'])},
                {'yaw': float(robot_pose['yaw'])}
            ]
        )

        agent_motion_planner: Node = Node(
            package='diff_drive_sim',
            executable='agent_motion_planner',
            name='agent_motion_planner',
            output='screen',
            parameters=[
                {'publish_rate': 10.0},
                {'goal_tolerance': 0.1},
                {'intermediate_goal_tolerance': 0.20}
            ]
        )

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(sim_pkg_share, 'rviz', 'rviz_view.rviz')]
        )

        return [
            gazebo,
            spawn_entities,
            bridge,
            pid_controller,
            agent_motion_planner,
            rviz
        ]

    return LaunchDescription([
        maze_arg,
        OpaqueFunction(function=launch_setup)
    ])