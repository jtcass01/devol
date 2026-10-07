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
    planner_pkg_share = get_package_share_directory('devol_local_planner')

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
    declare_planner = DeclareLaunchArgument(
        'planner',
        default_value='rrt_star',
        choices=['a_star', 'rrt', 'rrt_star'],
        description='Local planner: a_star (padded C-space map) or rrt / rrt_star (robot footprint on the octomap projected_map)',
    )
    declare_map_odom_tf = DeclareLaunchArgument(
        'map_odom_tf',
        default_value='static',
        choices=['static', 'none'],
        description='static: publish the placeholder map -> odom at the spawn pose; none: another node '
        'publishes map -> odom (e.g. devol_localization ground_truth_tf or a filter)',
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
        planner = context.perform_substitution(LaunchConfiguration('planner'))
        gz_gui = context.perform_substitution(LaunchConfiguration('gz_gui')) == 'true'
        use_rviz = context.perform_substitution(LaunchConfiguration('rviz')) == 'true'

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
            launch_arguments={
                'gz_args': f'-r {world_file}' if gz_gui else f'-r -s {world_file}'
            }.items(),
        )

        # Include spawn entities launch file (robot, goals, map, static transforms)
        spawn_entities = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg_share, 'launch', 'spawn_entities.launch.py')
            ),
            launch_arguments={
                'maze': maze_folder,
                'map_odom_tf': context.perform_substitution(LaunchConfiguration('map_odom_tf')),
            }.items(),
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
                # odom -> a200_base_link from the Gazebo DiffDrive plugin
                '/model/devol_drive/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/longeron_strut/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            ],
            remappings=[
                ('/model/devol_drive/tf', '/tf'),
                ('/tf_static', '/tf_static'),
                ('/model/devol_drive/pose', '/tf'),
                ('/model/longeron_strut/pose', '/tf'),
            ],
        )

        # Local planning stack (PID path follower + selected planner)
        local_planner = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planner_pkg_share, 'launch', 'local_planner.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'planner': planner,
                'x': robot_pose['x'],
                'y': robot_pose['y'],
                'yaw': robot_pose['yaw'],
            }.items(),
        )

        octomap_server = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server_node',
            output='screen',
            namespace=namespace,
            parameters=[
                {
                    'occupancy_min_z': octomap_resolution * 2,
                    'resolution': octomap_resolution,
                    'occupancy_max_z': 5.0,
                    'use_sim_time': True,
                }
            ],
            remappings=[('cloud_in', 'static_map_pointcloud')],
        )

        # Map publisher
        pointcloud_publisher = Node(
            package='devol_sim',
            executable='pointcloud_publisher',
            name='pointcloud_publisher',
            output='screen',
            parameters=[
                {
                    'pointcloud_file': pcd_file,
                    'frame_id': 'map',
                    'publish_rate': 1.0,  # octomap only needs the static cloud once
                    'topic_name': 'static_map_pointcloud',
                    'namespace': namespace,
                }
            ],
        )

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
            local_planner,
            octomap_server,
            pointcloud_publisher,
            goal_points_publisher,
        ] + ([rviz] if use_rviz else [])

    return LaunchDescription(
        [
            maze_arg,
            declare_namespace,
            declare_planner,
            DeclareLaunchArgument(
                'gz_gui',
                default_value='true',
                choices=['true', 'false'],
                description='Open the Gazebo GUI',
            ),
            DeclareLaunchArgument(
                'rviz', default_value='true', choices=['true', 'false'], description='Open RViz'
            ),
            declare_octomap_resolution,
            declare_map_odom_tf,
            OpaqueFunction(function=launch_setup),
        ]
    )
