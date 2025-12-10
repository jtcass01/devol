import os
import csv

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gz_pkg_share = get_package_share_directory('devol_gazebo')
    diff_drive_pkg_share = get_package_share_directory('diff_drive_description')
    urdf_package = "diff_drive_description"
    urdf_filename = "diff_drive.urdf.xacro"
    sdf_filename = 'diff_drive.sdf' # work around for mapping issues.
    pkg_share_description = FindPackageShare(urdf_package)
    urdf_path = PathJoinSubstitution(
        [pkg_share_description, "urdf", urdf_filename]
    )

    urdf_model = LaunchConfiguration("urdf_model")

    declare_urdf_model_cmd = DeclareLaunchArgument(
        "urdf_model",
        default_value=urdf_path,
        description="Path to the URDF model file",
    )

    robot_description_content: ParameterValue = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'tf_prefix:=diff_drive/'
    ]), value_type=str)

    robot_description = {'robot_description': robot_description_content,
                         'use_sim_time': True}
    
    # Declare maze selection argument
    maze_arg = DeclareLaunchArgument(
        'maze',
        default_value='Maze_ng',
        description='Maze to load: basic_maze or Maze_ng'
    )
    
    def launch_setup(context):
        maze_folder = context.perform_substitution(LaunchConfiguration('maze'))

        vehicle_model_file = os.path.join(diff_drive_pkg_share, 'urdf', sdf_filename)

        # Goal sphere SDF file
        goal_sphere_file = os.path.join(gz_pkg_share, 'sdf', 'goal_sphere.sdf')

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
        spawn_vehicle = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', vehicle_model_file,
                '-name', 'diff_drive',
                '-x', robot_pose['x'], 
                '-y', robot_pose['y'], 
                '-z', robot_pose['z'],
                '-Y', robot_pose['yaw']
            ],
            output='screen'
        )

        # Spawn goal spheres along x=0 behind each inner wall
        goal1_pose = poses['goal_1']
        spawn_goal1 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', goal_sphere_file,
                '-name', 'goal_1',
                '-x', goal1_pose['x'], '-y', goal1_pose['y'], '-z', goal1_pose['z']
            ],
            output='screen'
        )

        goal2_pose = poses['goal_2']
        spawn_goal2 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', goal_sphere_file,
                '-name', 'goal_2',
                '-x', goal2_pose['x'], '-y', goal2_pose['y'], '-z', goal2_pose['z']
            ],
            output='screen'
        )

        goal3_pose = poses['goal_3']
        spawn_goal3 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', goal_sphere_file,
                '-name', 'goal_3',
                '-x', goal3_pose['x'], '-y', goal3_pose['y'], '-z', goal3_pose['z']
            ],
            output='screen'
        )

        # Static transform publishers
        base_link_to_diff_drive_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0',  '0', '0',  '0', '0', 'diff_drive/base_link', 'base_link'],
            output='screen'
        )

        odom_to_diff_drive_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[robot_pose['x'], robot_pose['y'],  robot_pose['z'], robot_pose['yaw'],  '0', '0', 'map', 'odom'],
            output='screen'
        )

        maze_world_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'maze_world'],
            output='screen'
        )

        lidar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'diff_drive/lidar', 'diff_drive/base_link/lidar_sensor'],
            output='screen'
        )

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        # Map publisher
        map_publisher = Node(
            package='diff_drive_sim',
            executable='map_publisher',
            name='map_publisher',
            output='screen',
            parameters=[{'maze': maze_folder}]
        )

        # Goal points publisher
        goal_points_publisher = Node(
            package='diff_drive_sim',
            executable='goal_points_publisher',
            name='goal_points_publisher',
            output='screen',
            parameters=[{'maze': maze_folder}]
        )

        return [
            spawn_vehicle,
            spawn_goal1,
            spawn_goal2,
            spawn_goal3,
            odom_to_diff_drive_tf,
            base_link_to_diff_drive_tf,
            maze_world_tf,
            lidar_tf,
            robot_state_publisher,
            map_publisher,
            goal_points_publisher
        ]

    return LaunchDescription([
        declare_urdf_model_cmd,
        maze_arg,
        OpaqueFunction(function=launch_setup)
    ])
