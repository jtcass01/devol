from os.path import join
from typing import List
import csv

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

from devol_sim.mobile_robot_goal import MobileRobotGoal

def generate_launch_description():
    gz_pkg_share = get_package_share_directory('devol_gazebo')
    urdf_package = "devol_drive_description"
    urdf_filename = "devol_drive.urdf.xacro"
    moveit_package = "devol_moveit_config"

    pkg_share_description = FindPackageShare(urdf_package)
    urdf_path = PathJoinSubstitution(
        [pkg_share_description, "urdf", urdf_filename]
    )

    use_sim_time: LaunchConfiguration = LaunchConfiguration("use_sim_time")

    robot_description_content: ParameterValue = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), 
            ' ', urdf_path, ' ',
            "use_gazebo:=true ",
            "wheel_slip_compliance:=", LaunchConfiguration('wheel_slip_compliance'),
        ]
    ), value_type=str)

    robot_description = {'robot_description': robot_description_content,
                         'use_sim_time': True}

    # Declare maze selection argument
    maze_arg = DeclareLaunchArgument(
        'maze',
        default_value='moon_terrain',
        choices=["empty", "factory", "moon_terrain"],
        description='Maze to load: empty, factory or moon_terrain'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use Gazebo simulation clock",
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='/devol_drive',
        description='Namespace for topics'
    )
    declare_map_odom_tf = DeclareLaunchArgument(
        'map_odom_tf',
        default_value='static',
        choices=['static', 'none'],
        description='static: publish map -> odom at the spawn pose; none: leave it to another node'
    )
    declare_wheel_slip_compliance = DeclareLaunchArgument(
        'wheel_slip_compliance',
        default_value='0.5',
        description='Unitless WheelSlip compliance for all four wheels (lateral and longitudinal); 0 = no slip'
    )
    declare_publish_robot_description_semantic_cmd = DeclareLaunchArgument(
        "publish_robot_description_semantic",
        default_value="true",
        choices=["true", "false"],
        description="Publish the robot description semantic",
    )
    
    def launch_setup(context):
        maze_folder = context.perform_substitution(LaunchConfiguration('maze'))
        namespace = context.perform_substitution(LaunchConfiguration('namespace'))

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
                    g0: MobileRobotGoal = MobileRobotGoal(name=row['name'], x=float(row['x']),
                                         y=float(row['y']), z=float(row['z']),
                                         yaw=float(row['yaw']))
                else:
                    goals.append(MobileRobotGoal(name=row['name'], x=float(row['x']),
                                                 y=float(row['y']), z=float(row['z']),
                                                 yaw=float(row['yaw'])))

        # Spawn the vehicle model in Gazebo
        spawn_robot: Node = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-topic", f'{namespace}/robot_description', 
                       '-name', 'devol_drive', 
                       '-x', str(g0.x), 
                       '-y', str(g0.y), 
                       '-z', str(g0.z), 
                       '-Y', str(g0.yaw), 
                       '-allow_renaming', 'true'],
            parameters=[{"use_sim_time": use_sim_time}]
        )

        # Spawn goal spheres along x=0 behind each inner wall
        goal_spawners: List[Node] = []
        for goal in goals:
            goal: MobileRobotGoal = goal
            spawn_goal = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', goal_sphere_file,
                    '-name', goal.name,
                    '-x', str(goal.x), 
                    '-y', str(goal.y), 
                    '-z', str(goal.z)
                ],
                output='screen'
            )
            goal_spawners.append(spawn_goal)

        # MoveIt
        moveit_config = (
            MoveItConfigsBuilder(robot_name="devol", package_name=moveit_package)
            .to_moveit_configs()
        )

        start_move_group_cmd: Node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace=namespace,
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_description_content,
                    "publish_robot_description_semantic": LaunchConfiguration(
                        "publish_robot_description_semantic"
                    ),
                }
            ]
        )

        # Static transform publishers
        base_link_to_diff_drive_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0',  '0', '0',  '0', '0', f'{namespace}/a200_base_link', 'a200_base_link'],
            output='screen'
        )

        odom_to_diff_drive_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[str(g0.x), str(g0.y),  str(g0.z), str(g0.yaw),  '0', '0', 'map', 'odom'],
            output='screen'
        )

        maze_world_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'maze_world'],
            output='screen'
        )

        lidar2d_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar2d_0_link', f'{namespace}/robot/base_link/lidar2d_0'],
            output='screen'
        )

        lidar3d_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar3d_0_link', f'{namespace}/robot/base_link/lidar3d_0'],
            output='screen'
        )

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[robot_description,
                        {"use_sim_time": use_sim_time}]
        )

        return [
            spawn_robot,
            *goal_spawners,
            base_link_to_diff_drive_tf,
            *([odom_to_diff_drive_tf]
              if context.perform_substitution(LaunchConfiguration('map_odom_tf')) == 'static' else []),
            maze_world_tf,
            lidar2d_tf,
            lidar3d_tf,
            robot_state_publisher,
            start_move_group_cmd
        ]

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_namespace,
        declare_publish_robot_description_semantic_cmd,
        maze_arg,
        declare_map_odom_tf,
        declare_wheel_slip_compliance,
        OpaqueFunction(function=launch_setup)
    ])
