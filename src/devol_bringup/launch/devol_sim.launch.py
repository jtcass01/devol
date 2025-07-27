from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
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
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory



ARGUMENTS = [
    DeclareLaunchArgument('name', default_value='devol',
                          description=''),
]


def generate_launch_description():
    # Define file names
    urdf_package = "devol_description"
    moveit_package = "devol_moveit_config"
    project_gz_package_name: str = "devol_gazebo"
    gz_package_name: str = "ros_gz_sim"

    gz_launch_filename: str = "gz_sim.launch.py"
    urdf_filename = "devol.urdf.xacro"
    rviz_config_filename = "moveit.rviz"

    # Define paths
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_path = PathJoinSubstitution(
        [pkg_share_description, "urdf", urdf_filename]
    )
    pkg_share_moveit = FindPackageShare(moveit_package)
    default_rviz_path = PathJoinSubstitution(
        [pkg_share_moveit, "config", rviz_config_filename]
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
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )
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
        default_value="factory",
        choices=["empty", "factory"]
    )

    robot_description_content: Command = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            ' ', default_urdf_path, ' ',
            'use_gazebo:=true '

        ]
    )

    robot_description = {'robot_description': robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder(robot_name="devol", package_name=moveit_package)
        # .robot_description_semantic(file_path=join(get_package_share_directory(moveit_package),
        #                                       "config",
        #                                       "devol.srdf"))
        # .robot_description(file_path=join(get_package_share_directory(moveit_package),
        #                              "config",
        #                              urdf_filename))
        # .joint_limits(file_path=join(get_package_share_directory(moveit_package),
        #                              "config",
        #                              "joint_limits.yaml"))
        # .trajectory_execution(file_path=join(get_package_share_directory(moveit_package),
        #                                 "config",
        #                                 "moveit_controllers.yaml"))
        .to_moveit_configs()
    )

    # Subscribe to the joint states of the robot, and publish them to the robot state publisher
    start_robot_state_publisher_cmd: Node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description])


    # Spawn robot command
    gz_spawn_entity_cmd: Node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", 'robot_description', '-name', 'devol', '-x', '0', '-y', '0', '-z', '0.3', '-allow_renaming', 'true']
    )

    start_move_group_cmd: Node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_content,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            }
        ]
    )

    # Launch RViz
    start_rviz_cmd: Node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='devol_rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time}],
    )

    start_gz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r -v 4 '),
                PathJoinSubstitution([
                    project_world_directory, world_directory, 'model.sdf'
                ])
            ]
        }.items(),
    )

    declare_gz_sim_resource_path_env_var = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution(
            [project_world_directory, world_directory, 
             ":", FindPackageShare("ur_description"), 
             ":", FindPackageShare("robotiq_description"), 
             ":", FindPackageShare("devol_description"), 
             ":$GZ_SIM_RESOURCE_PATH"])
    )

    start_gz_bridge_cmd: Node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                   "/model/devol/color_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/model/devol/color_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                   "/model/devol/depth_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/model/devol/depth_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                   ],
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
    ld.add_action(start_move_group_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(declare_gz_sim_resource_path_env_var)
    ld.add_action(start_gz_cmd)
    ld.add_action(start_gz_bridge_cmd)

    return ld