"""Factory (or another world) in Gazebo with the Devol robot, the octomap map pipeline and a local planner.

  ros2 launch devol_sim motion_planner_sim.launch.py rviz:=false
  ros2 launch devol_sim motion_planner_sim.launch.py planner:=none     # sim and map only, robot stays put

Starts Gazebo on devol_gazebo/worlds/<maze>/maze_world.sdf, spawns the robot at the 'robot' row of that
world's poses.csv and a goal sphere at every other row, bridges the robot's topics, builds the map
(static_world.pcd -> octomap_server -> <namespace>/projected_map) and drives the robot through the goals
with devol_local_planner. devol_localization's localization_sim.launch.py includes this file.
"""

import csv
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'maze': (
        'factory',
        'World folder in devol_gazebo/worlds: factory, lunar, empty or moon_terrain',
    ),
    'namespace': ('/devol_drive', 'Namespace for the robot topics'),
    'planner': (
        'rrt_star',
        'a_star (padded C-space map), rrt / rrt_star (robot footprint on projected_map), '
        'or none (no planner or path follower)',
    ),
    'map_odom_tf': (
        'static',
        'static: publish a placeholder map -> odom at the spawn pose; none: another node publishes it '
        '(devol_localization ground_truth_tf or a filter)',
    ),
    'octomap_resolution': ('0.05', 'Octomap voxel size in m'),
    'gz_gui': ('true', 'Open the Gazebo GUI (false runs the server headless)'),
    'rviz': ('true', 'Open RViz'),
    'use_cameras': ('false', 'Simulate the RGB-D cameras (costly; not needed for localization)'),
    'wheel_slip_compliance': (
        '0.5',
        'Unitless WheelSlip compliance for all four wheels (lateral and longitudinal); 0 = no slip',
    ),
}


def static_tf(parent, child, x='0', y='0', z='0', yaw='0'):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--x', x, '--y', y, '--z', z, '--yaw', yaw]
        + ['--frame-id', parent, '--child-frame-id', child],
    )


def launch_setup(context):
    a = {name: LaunchConfiguration(name).perform(context) for name in ARGS}
    ns = a['namespace']
    gz_share = get_package_share_directory('devol_gazebo')
    world_dir = os.path.join(gz_share, 'worlds', a['maze'])
    with open(os.path.join(world_dir, 'poses.csv')) as f:
        poses = {row['name']: row for row in csv.DictReader(f)}
    robot = poses.pop('robot')

    gz_args = ('-r ' if a['gz_gui'] == 'true' else '-r -s ') + os.path.join(
        world_dir, 'maze_world.sdf'
    )
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'robot_description': ParameterValue(
                        Command(
                            [
                                FindExecutable(name='xacro'),
                                ' ',
                                os.path.join(
                                    get_package_share_directory('devol_drive_description'),
                                    'urdf',
                                    'devol_drive.urdf.xacro',
                                ),
                                f' use_gazebo:=true use_cameras:={a["use_cameras"]}',
                                f' wheel_slip_compliance:={a["wheel_slip_compliance"]}',
                            ]
                        ),
                        value_type=str,
                    ),
                }
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-topic', f'{ns}/robot_description', '-name', 'devol_drive']
            + ['-x', robot['x'], '-y', robot['y'], '-z', robot['z'], '-Y', robot['yaw']]
            + ['-allow_renaming', 'true'],
        ),
    ]
    for name, goal in poses.items():
        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-file',
                    os.path.join(gz_share, 'sdf', 'goal_sphere.sdf'),
                    '-name',
                    name,
                ]
                + ['-x', goal['x'], '-y', goal['y'], '-z', goal['z']],
            )
        )

    # Alias for nodes that look up '<namespace>/a200_base_link' (planners, PID); odom -> a200_base_link
    # comes from the Gazebo DiffDrive plugin.
    actions.append(static_tf('a200_base_link', f'{ns.lstrip("/")}/a200_base_link'))
    if a['map_odom_tf'] == 'static':
        actions.append(static_tf('map', 'odom', robot['x'], robot['y'], robot['z'], robot['yaw']))
    actions.append(static_tf('map', 'maze_world'))

    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('devol_drive_description'),
                    'launch',
                    'ros_gz_bridge.launch.py',
                )
            ),
            launch_arguments={'namespace': ns}.items(),
        ),
        Node(
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
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/devol_drive/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                # odom -> a200_base_link from the Gazebo DiffDrive plugin
                '/model/devol_drive/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[('/model/devol_drive/tf', '/tf'), ('/model/devol_drive/pose', '/tf')],
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server_node',
            output='screen',
            namespace=ns,
            parameters=[
                {
                    'occupancy_min_z': float(a['octomap_resolution']) * 2,
                    'resolution': float(a['octomap_resolution']),
                    'occupancy_max_z': 5.0,
                    'use_sim_time': True,
                }
            ],
            remappings=[('cloud_in', 'static_map_pointcloud')],
        ),
        Node(
            package='devol_sim',
            executable='pointcloud_publisher',
            name='pointcloud_publisher',
            output='screen',
            parameters=[
                {
                    'pointcloud_file': os.path.join(world_dir, 'static_world.pcd'),
                    'frame_id': 'map',
                    'publish_rate': 1.0,  # octomap only needs the static cloud once
                    'topic_name': 'static_map_pointcloud',
                    'namespace': ns,
                }
            ],
        ),
        Node(
            package='devol_sim',
            executable='goal_points_publisher',
            name='goal_points_publisher',
            output='screen',
            parameters=[{'maze': a['maze'], 'namespace': ns}],
        ),
    ]

    if a['planner'] != 'none':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('devol_local_planner'),
                        'launch',
                        'local_planner.launch.py',
                    )
                ),
                launch_arguments={
                    'namespace': ns,
                    'planner': a['planner'],
                    'x': robot['x'],
                    'y': robot['y'],
                    'yaw': robot['yaw'],
                }.items(),
            )
        )
    if a['rviz'] == 'true':
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d',
                    os.path.join(
                        get_package_share_directory('devol_sim'), 'rviz', 'rviz_view.rviz'
                    ),
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, (d, h) in ARGS.items()]
        + [OpaqueFunction(function=launch_setup)]
    )
