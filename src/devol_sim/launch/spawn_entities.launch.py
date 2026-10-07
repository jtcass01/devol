from os.path import join
from typing import List
import csv

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

from devol_local_planner.mobile_robot_goal import MobileRobotGoal


def generate_launch_description():
    gz_pkg_share = get_package_share_directory('devol_gazebo')
    urdf_package = 'devol_drive_description'
    urdf_filename = 'devol_drive.urdf.xacro'

    pkg_share_description = FindPackageShare(urdf_package)
    urdf_path = PathJoinSubstitution([pkg_share_description, 'urdf', urdf_filename])

    use_sim_time: LaunchConfiguration = LaunchConfiguration('use_sim_time')

    robot_description_content: ParameterValue = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                urdf_path,
                ' ',
                'use_gazebo:=true ',
                'use_cameras:=',
                LaunchConfiguration('use_cameras'),
                ' wheel_slip_compliance:=',
                LaunchConfiguration('wheel_slip_compliance'),
            ]
        ),
        value_type=str,
    )

    robot_description = {'robot_description': robot_description_content, 'use_sim_time': True}

    # Declare maze selection argument
    maze_arg = DeclareLaunchArgument(
        'maze',
        default_value='moon_terrain',
        choices=['empty', 'factory', 'moon_terrain'],
        description='Maze to load: empty, factory or moon_terrain',
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use Gazebo simulation clock',
    )
    declare_use_cameras_cmd = DeclareLaunchArgument(
        'use_cameras',
        default_value='false',
        choices=['true', 'false'],
        description='Simulate the RGB-D cameras (costly; not needed for localization)',
    )
    declare_map_odom_tf = DeclareLaunchArgument(
        'map_odom_tf',
        default_value='static',
        choices=['static', 'none'],
        description='static: publish map -> odom at the spawn pose; none: leave it to another node',
    )
    declare_wheel_slip_compliance = DeclareLaunchArgument(
        'wheel_slip_compliance',
        default_value='0.5',
        description='Unitless WheelSlip compliance for all four wheels (lateral and '
        'longitudinal); 0 = no slip',
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='/devol_drive', description='Namespace for topics'
    )

    def launch_setup(context):
        maze_folder = context.perform_substitution(LaunchConfiguration('maze'))
        namespace = context.perform_substitution(LaunchConfiguration('namespace'))
        static_map_odom = (
            context.perform_substitution(LaunchConfiguration('map_odom_tf')) == 'static'
        )

        # Goal sphere SDF file
        goal_sphere_file = join(gz_pkg_share, 'sdf', 'goal_sphere.sdf')

        # Poses CSV file
        poses_file = join(gz_pkg_share, 'worlds', maze_folder, 'poses.csv')

        # Read poses from CSV
        goals: List = []
        with open(poses_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row['name'] == 'robot':
                    g0: MobileRobotGoal = MobileRobotGoal(
                        name=row['name'],
                        x=float(row['x']),
                        y=float(row['y']),
                        z=float(row['z']),
                        yaw=float(row['yaw']),
                    )
                else:
                    goals.append(
                        MobileRobotGoal(
                            name=row['name'],
                            x=float(row['x']),
                            y=float(row['y']),
                            z=float(row['z']),
                            yaw=float(row['yaw']),
                        )
                    )

        # Spawn the vehicle model in Gazebo
        spawn_robot: Node = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic',
                f'{namespace}/robot_description',
                '-name',
                'devol_drive',
                '-x',
                str(g0.x),
                '-y',
                str(g0.y),
                '-z',
                str(g0.z),
                '-Y',
                str(g0.yaw),
                '-allow_renaming',
                'true',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        # Spawn goal spheres along x=0 behind each inner wall
        goal_spawners: List[Node] = []
        for goal in goals:
            goal: MobileRobotGoal = goal
            spawn_goal = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file',
                    goal_sphere_file,
                    '-name',
                    goal.name,
                    '-x',
                    str(goal.x),
                    '-y',
                    str(goal.y),
                    '-z',
                    str(goal.z),
                ],
                output='screen',
            )
            goal_spawners.append(spawn_goal)

        # Static transform publishers
        base_link_to_diff_drive_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            # Alias for nodes that look up '<namespace>/a200_base_link' (planners, PID); odom -> a200_base_link comes from Gazebo DiffDrive.
            arguments=[
                '--frame-id',
                'a200_base_link',
                '--child-frame-id',
                f'{namespace.lstrip("/")}/a200_base_link',
            ],
            output='screen',
        )

        odom_to_diff_drive_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x',
                str(g0.x),
                '--y',
                str(g0.y),
                '--z',
                str(g0.z),
                '--yaw',
                str(g0.yaw),
                '--frame-id',
                'map',
                '--child-frame-id',
                'odom',
            ],
            output='screen',
        )

        maze_world_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'maze_world'],
            output='screen',
        )

        lidar2d_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--frame-id',
                'lidar2d_0_link',
                '--child-frame-id',
                f'{namespace}/robot/base_link/lidar2d_0',
            ],
            output='screen',
        )

        lidar3d_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--frame-id',
                'lidar3d_0_link',
                '--child-frame-id',
                f'{namespace}/robot/base_link/lidar3d_0',
            ],
            output='screen',
        )

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        )

        return (
            [
                spawn_robot,
                *goal_spawners,
                base_link_to_diff_drive_tf,
            ]
            + ([odom_to_diff_drive_tf] if static_map_odom else [])
            + [maze_world_tf, lidar2d_tf, lidar3d_tf, robot_state_publisher]
        )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_use_cameras_cmd,
            declare_map_odom_tf,
            declare_wheel_slip_compliance,
            declare_namespace,
            maze_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
