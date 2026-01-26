from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    TextSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Define file names
    urdf_package = "a200_description"
    urdf_pkg_share = get_package_share_directory(urdf_package)
    project_gz_package_name: str = "devol_gazebo"
    gz_package_name: str = "ros_gz_sim"

    gz_launch_filename: str = "gz_sim.launch.py"
    urdf_filename = "a200.urdf.xacro"
    rviz_config_filename = "a200.rviz"

    # Define paths
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_path = PathJoinSubstitution(
        [pkg_share_description, "urdf", urdf_filename]
    )
    default_rviz_path = PathJoinSubstitution(
        [pkg_share_description, "config", rviz_config_filename]
    )
    project_gz_path: FindPackageShare = FindPackageShare(project_gz_package_name)
    project_world_directory: PathJoinSubstitution = PathJoinSubstitution(
        [project_gz_path, "worlds"]
    )
    gz_path: FindPackageShare = FindPackageShare(gz_package_name)
    gz_launch_path: PathJoinSubstitution = PathJoinSubstitution(
        [gz_path, "launch", gz_launch_filename]
    )

    # Launch configuration variables
    world_directory = LaunchConfiguration("world")

    # Declare launch arguments
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_path,
        description="Path to RViz config file",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch RViz with the robot description",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use Gazebo simulation clock",
    )
    declare_publish_robot_description_semantic_cmd = DeclareLaunchArgument(
        "publish_robot_description_semantic",
        default_value="true",
        choices=["true", "false"],
        description="Publish the robot description semantic",
    )
    declare_world_directory_cmd = DeclareLaunchArgument(
        "world",
        default_value="empty",
        choices=['empty', "basic_maze", "Maze_hr", "Maze_ng", "Maze_ql_1"]
    )

    robot_description_content: ParameterValue = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), ' ', default_urdf_path, ' '
        ]
    ), value_type=str)

    robot_description = {'robot_description': robot_description_content,
                         'use_sim_time': True}

    # Subscribe to the joint states of the robot, and publish them to the robot state publisher
    start_robot_state_publisher_cmd: Node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='a200_0000',
        output='screen',
        parameters=[robot_description],
        remappings=[
            ('/joint_states', 'dynamic_joint_states')
        ])

    lidar2d_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar2d_0_link', 'a200_0000/robot/base_link/lidar2d_0'],
        output='screen'
    )

    # Spawn robot command
    gz_spawn_entity_cmd: Node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", '/a200_0000/robot_description', '-name', 'a200', '-x', '0', '-y', '0', '-z', '0.2']
    )

    start_gz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r -v 4 '),
                PathJoinSubstitution([
                    project_world_directory, world_directory, 'maze_world.sdf'
                ]),
            ]
        }.items(),
    )

    declare_gz_sim_resource_path_env_var = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution(
            [project_world_directory, world_directory, 
             ":", FindPackageShare("clearpath_platform_description"), 
             ":", FindPackageShare("velodyne_description"), 
             ":", FindPackageShare("realsense2_description"), 
             ":$GZ_SIM_RESOURCE_PATH"])
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', join(urdf_pkg_share, 'rviz', 'a200.rviz')]
    )

    start_system_bridge_cmd = Node(
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
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
           '/model/a200/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
           ('/model/a200_0000/tf', '/tf'),
           ('/tf_static', '/tf_static'),
           ('/model/a200/pose', '/tf'),
       ]
    )

    start_a200_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(urdf_pkg_share, 'launch', 'ros_gz_bridge.launch.py')
        )
    )

    # Create the launch description and populate with arguments
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_directory_cmd)
    ld.add_action(declare_publish_robot_description_semantic_cmd)

    # Add actions
    ld.add_action(gz_spawn_entity_cmd)
    ld.add_action(lidar2d_tf)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(declare_gz_sim_resource_path_env_var)
    ld.add_action(start_gz_cmd)
    ld.add_action(start_system_bridge_cmd)
    ld.add_action(start_a200_bridge_cmd)
    ld.add_action(rviz)

    return ld