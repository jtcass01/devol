from os.path import join
import csv

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gz_pkg_share = get_package_share_directory('devol_gazebo')
    urdf_package = "devol_drive_description"
    urdf_filename = "devol_drive.urdf.xacro"

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
        ]
    ), value_type=str)

    robot_description = {'robot_description': robot_description_content,
                         'use_sim_time': True}
    
    # Declare maze selection argument
    maze_arg = DeclareLaunchArgument(
        'maze',
        default_value='factory',
        description='Maze to load: basic_maze or Maze_ng'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use Gazebo simulation clock",
    )
    
    def launch_setup(context):
        maze_folder = context.perform_substitution(LaunchConfiguration('maze'))

        # Goal sphere SDF file
        goal_sphere_file = join(gz_pkg_share, 'sdf', 'goal_sphere.sdf')

        # Poses CSV file
        poses_file = join(gz_pkg_share, 'worlds', maze_folder, 'poses.csv')

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
        spawn_robot: Node = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-topic", '/a200_0000/robot_description', '-name', 'devol_drive', '-x', robot_pose['x'], '-y', robot_pose['y'], '-z', robot_pose['z'], '-Y', robot_pose['yaw'], '-allow_renaming', 'true'],
            parameters=[{"use_sim_time": use_sim_time}]
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
            arguments=['0', '0',  '0', '0',  '0', '0', 'devol_drive/a200_base_link', 'a200_base_link'],
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

        lidar2d_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar2d_0_link', 'devol_drive/robot/base_link/lidar2d_0'],
            output='screen'
        )

        lidar3d_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar3d_0_link', 'devol_drive/robot/base_link/lidar3d_0'],
            output='screen'
        )

        # Robot state publisher
        robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='a200_0000',
        output='screen',
        parameters=[robot_description,
                    {"use_sim_time": use_sim_time}])
        
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
            spawn_robot,
            spawn_goal1,
            spawn_goal2,
            spawn_goal3,
            base_link_to_diff_drive_tf,
            odom_to_diff_drive_tf,
            maze_world_tf,
            lidar2d_tf,
            lidar3d_tf,
            robot_state_publisher,
            map_publisher,
            goal_points_publisher
        ]

    return LaunchDescription([
        declare_use_sim_time_cmd,
        maze_arg,
        OpaqueFunction(function=launch_setup)
    ])
