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


ARGUMENTS = []


def generate_launch_description():
    # Define file names
    urdf_package = "devol_drive_description"
    project_gz_package_name: str = "devol_gazebo"
    gz_package_name: str = "ros_gz_sim"

    gz_launch_filename: str = "gz_sim.launch.py"
    urdf_filename = "devol_drive.urdf.xacro"
    rviz_config_filename = "devol_drive.rviz"

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
    use_sim_time: LaunchConfiguration = LaunchConfiguration("use_sim_time")

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
        default_value="factory",
        choices=["empty", "factory"]
    )

    robot_description_content: ParameterValue = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), 
            ' ', default_urdf_path, ' ',
            "use_gazebo:=true ",
        ]
    ), value_type=str)

    robot_description = {'robot_description': robot_description_content,
                         'use_sim_time': True}

    # Subscribe to the joint states of the robot, and publish them to the robot state publisher
    start_robot_state_publisher_cmd: Node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description,
                    {"use_sim_time": use_sim_time}])


    # Spawn robot command
    gz_spawn_entity_cmd: Node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", 'robot_description', '-name', 'devol_drive', '-x', '0', '-y', '0', '-z', '0', '-allow_renaming', 'true'],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    start_gz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r -v 4 '),
                PathJoinSubstitution([
                    project_world_directory, world_directory, 'model.sdf'
                ]),
            ]
        }.items(),
    )

    declare_gz_sim_resource_path_env_var = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution(
            [project_world_directory, world_directory, 
             ":", FindPackageShare("devol_drive_description"), 
             ":$GZ_SIM_RESOURCE_PATH"])
    )

    start_gz_bridge_cmd: Node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'qos_overrides./model/devol_drive.subscriber.reliability': 'reliable',
                     "use_sim_time": use_sim_time}],
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                   "/model/devol_drive/color_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/model/devol_drive/color_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                   "/model/devol_drive/depth_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/model/devol_drive/depth_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                   '/model/devol_drive/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/devol_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )

    # Create the launch description and populate with arguments
    ld = LaunchDescription(ARGUMENTS)

    # Declare the launch options
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_directory_cmd)
    ld.add_action(declare_publish_robot_description_semantic_cmd)

    # Add actions
    ld.add_action(gz_spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(declare_gz_sim_resource_path_env_var)
    ld.add_action(start_gz_cmd)
    ld.add_action(start_gz_bridge_cmd)

    return ld