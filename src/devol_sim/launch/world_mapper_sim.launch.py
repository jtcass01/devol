import os
import csv

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    devol_drive_pkg_share = get_package_share_directory('devol_drive_description')
    sim_pkg_share = get_package_share_directory('devol_sim')
    gz_pkg_share = get_package_share_directory('devol_gazebo')

    # Declare maze selection argument
    maze_arg = DeclareLaunchArgument(
        'maze',
        default_value='factory',
        choices=['empty', 'factory', 'moon_terrain'],
        description='Maze to load: empty, factory or moon_terrain',
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='/devol_drive', description='Namespace for topics'
    )
    declare_octomap_resolution = DeclareLaunchArgument(
        'octomap_resolution', default_value='0.05', description='Namespace for topics'
    )

    def launch_setup(context):
        maze_folder = context.perform_substitution(LaunchConfiguration('maze'))
        namespace = context.perform_substitution(LaunchConfiguration('namespace'))
        octomap_resolution = float(
            context.perform_substitution(LaunchConfiguration('octomap_resolution'))
        )

        world_file = os.path.join(gz_pkg_share, 'worlds', maze_folder, 'maze_world.sdf')

        # Poses CSV file
        poses_file = os.path.join(gz_pkg_share, 'worlds', maze_folder, 'poses.csv')

        # Static Map File
        pcd_file: str = os.path.join(gz_pkg_share, 'worlds', maze_folder, 'static_world.pcd')

        # Read poses from CSV
        poses = {}
        with open(poses_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                poses[row['name']] = {
                    'x': row['x'],
                    'y': row['y'],
                    'z': row['z'],
                    'yaw': row['yaw'],
                }

        # Spawn the vehicle model in Gazebo
        robot_pose = poses['robot']

        # Launch Gazebo with the world
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )
                ]
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        )

        # Include spawn entities launch file (robot, goals, map, static transforms)
        spawn_entities = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg_share, 'launch', 'spawn_entities.launch.py')
            ),
            launch_arguments={'maze': maze_folder}.items(),
        )

        # Bridge, PID, RViz
        bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(devol_drive_pkg_share, 'launch', 'ros_gz_bridge.launch.py')
            ),
            launch_arguments={'namespace': namespace}.items(),
        )

        system_bridge_cmd = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_system',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'qos_overrides./tf.publisher.durability': 'transient_local',
                    'qos_overrides./tf_static.publisher.durability': 'transient_local',
                }
            ],
            arguments=[
                # -----------------
                # Simulation clock
                # -----------------
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/devol_drive/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            ],
            remappings=[
                ('/model/devol_drive/tf', '/tf'),
                ('/tf_static', '/tf_static'),
                ('/model/devol_drive/pose', '/tf'),
            ],
        )

        pid_controller = Node(
            package='devol_sim',
            executable='diffdrive_pid',
            name='diffdrive_pid',
            output='screen',
            parameters=[
                {
                    'kp': 0.9,
                    'kd': 0.0,
                    'ki': 0.0,
                    'lookahead': 0.3,
                    'publish_rate': 30.0,
                    'max_linear_vel': 0.75,
                    'max_angular_vel': 1.0,
                    'x': float(robot_pose['x']),
                    'y': float(robot_pose['y']),
                    'yaw': float(robot_pose['yaw']),
                    'namespace': namespace,
                }
            ],
        )

        octomap_server = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server_node',
            output='screen',
            namespace=namespace,
            parameters=[
                {
                    'occupancy_min_z': 0.0,
                    'resolution': octomap_resolution,
                    'occupancy_max_z': 5.0,
                    'use_sim_time': True,
                }
            ],
            remappings=[
                (
                    'cloud_in',
                    'static_map_pointcloud',
                )  # These topics are namespaced! You will see them on ros2 topics as
                # $NAMESPACE/static_map_pointcloud or /devol_drive/static_map_pointcloud
                # for our sim.
            ],
        )

        # Pointcloud publisher -- ADD YOUR PUBLISHER IN THIS PLACE! Note the topic name listed here is namespaced.
        # pointcloud_publisher = Node(
        #     package='devol_sim',
        #     executable='pointcloud_publisher',
        #     name='pointcloud_publisher',
        #     output='screen',
        #     parameters=[{'pointcloud_file': pcd_file,
        #                  'frame_id': 'map',
        #                  'publish_rate': 30.0,
        #                  'topic_name': 'static_map_pointcloud',
        #                  'namespace': namespace}]
        # )

        # Goal points publisher
        goal_points_publisher = Node(
            package='devol_sim',
            executable='goal_points_publisher',
            name='goal_points_publisher',
            output='screen',
            parameters=[{'maze': maze_folder, 'namespace': namespace}],
        )

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(sim_pkg_share, 'rviz', 'rviz_view.rviz')],
        )

        return [
            gazebo,
            spawn_entities,
            bridge,
            system_bridge_cmd,
            pid_controller,
            octomap_server,
            # pointcloud_publisher, ADD YOUR MAP PUBLISHER HERE TOO
            goal_points_publisher,
            rviz,
        ]

    return LaunchDescription(
        [
            maze_arg,
            declare_namespace,
            declare_octomap_resolution,
            OpaqueFunction(function=launch_setup),
        ]
    )
