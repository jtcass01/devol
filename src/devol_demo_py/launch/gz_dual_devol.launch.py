from typing import Final, List, Dict

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)
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
from moveit_configs_utils import MoveItConfigsBuilder

from devol_demo_py.collision_checker import MultiRobotCollisionServer
from devol_demo_py.world_state_publisher import WorldStatePublisher


GAZEBO_CAMERA_START_POSE: Final[str] = (
    "pose: {"
    "position { x: 0 y: 0 z: 6 } "
    "orientation { x: 0 y: 1 z: 0 w: 1 }}"
)


def generate_launch_description():
    # Define file names
    urdf_package = "devol_drive_description"
    project_gz_package_name: str = "arc_lsi_gazebo"
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
    publish_robot_description_semantic = LaunchConfiguration(
        "publish_robot_description_semantic"
    )

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

    robot_configs: List[Dict[str, str]] = [
        {"name": "apollo", 'x': '0.0', 'y': '-1.0', 'z': '0.3'},
        {"name": "artemis", 'x': '0.0', 'y': '1.0', 'z': '0.3'}
    ]
    robot_nodes: List[Node] = []
    for robot_config in robot_configs:
        robot_name: str = robot_config['name']
        tf_prefix: str = f"{robot_name}_"

        robot_description_content: ParameterValue = ParameterValue(Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]), 
                ' ', default_urdf_path, ' ',
                f'name:={robot_name} ',
                f'tf_prefix:={tf_prefix} ',
                "use_gazebo:=true "
            ]
        ), value_type=str)

        robot_description = {'robot_description': robot_description_content,
                            'use_sim_time': True}
        
        # Subscribe to the joint states of the robot, and publish them to the robot state publisher
        start_robot_state_publisher_cmd: Node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[robot_description,
                        {"use_sim_time": use_sim_time}])

        # Spawn robot command
        gz_spawn_entity_cmd: Node = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic", f'/{robot_name}/robot_description', 
                '-name', robot_name, 
                '-x', robot_config['x'], 
                '-y', robot_config['y'], 
                '-z', robot_config['z'], 
                '-allow_renaming', 'true'],
            parameters=[{"use_sim_time": use_sim_time}]
        )

        moveit_config = (
            MoveItConfigsBuilder(robot_name=robot_name, package_name=f"{tf_prefix}moveit_config")
            .to_moveit_configs()
        )

        start_move_group_cmd: Node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace=robot_name,
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_description_content,
                    "publish_robot_description_semantic": publish_robot_description_semantic,
                    "start_state_monitor": False,
                    "monitor_diffs": False,
                }
            ]
        )

        robot_nodes.extend([start_robot_state_publisher_cmd, gz_spawn_entity_cmd, start_move_group_cmd])

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
             ":", FindPackageShare("ur_description"), 
             ":", FindPackageShare("robotiq_description"), 
             ":", FindPackageShare("devol_description"), 
             ":", FindPackageShare("devol_drive_description"), 
             ":$GZ_SIM_RESOURCE_PATH"])
    )

    bridge_arguments: List[str] = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
    ]

    target_blocks: int = 3
    camera_channels: List[str] = ['color', 'depth']
    for robot_config in robot_configs:
        robot_name: str = robot_config['name']
        tf_prefix: str = f"{robot_name}_"

        # Target block topics
        target_block_topics: List[str] = []
        for target_block in range(target_blocks):
            # Attach topic
            target_block_topics.append(f"/{tf_prefix}attach_{target_block}/attach@std_msgs/msg/Empty]gz.msgs.Empty")
            # Detach topic
            target_block_topics.append(f"/{tf_prefix}attach_{target_block}/detach@std_msgs/msg/Empty]gz.msgs.Empty")
        bridge_arguments.extend(target_block_topics)

        # Camera topics
        camera_topics: List[str] = []
        for camera_channel in camera_channels:
            # Raw Image Channel
            camera_topics.append(f"/model/{tf_prefix}camera/{camera_channel}_raw@sensor_msgs/msg/Image[gz.msgs.Image")
            # Camera Info Channel
            camera_topics.append(f"/model/{tf_prefix}camera/{camera_channel}_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo")
        bridge_arguments.extend(camera_topics)

        # Drive topics
        drive_topics: List[str] = []
        drive_topics.append(f"/model/{tf_prefix}drive/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist")
        drive_topics.append(f"/model/{tf_prefix}drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry")
        bridge_arguments.extend(drive_topics)

        # Pose topic
        bridge_arguments.append(f"/model/{robot_name}/tf@/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")
        bridge_arguments.append(f"/model/{robot_name}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose")

    start_gz_bridge_cmd: Node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'qos_overrides./model/devol_drive.subscriber.reliability': 'reliable',
                     "use_sim_time": use_sim_time}],
        arguments=bridge_arguments,
        output='screen'
    )

    # Create ARC-LSI Nodes
    world_state_publisher_node = Node(
        package='devol_demo_py',  # Your package name
        executable='world_state_publisher',  # Your executable name
        name='world_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    collision_server_node = Node(
        package='devol_demo_py',  # Your package name
        executable='collision_checker',  # Your executable name
        name='multi_robot_collision_server',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_names": [robot_config['name'] for robot_config in robot_configs]}
        ]
    )

    arc_lsi_nodes: List[Node] = [
        world_state_publisher_node,
        collision_server_node
    ]

    # Delay camera move for 5 seconds to give time for Gazebo to create the GUI services
    set_gz_initial_gui_camera_position = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    PathJoinSubstitution([FindExecutable(name="gz")]),
                    "service",
                    "-s",
                    "/gui/move_to/pose",
                    "--reqtype",
                    "gz.msgs.GUICamera",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "2000",
                    "--req",
                    GAZEBO_CAMERA_START_POSE,
                ],
                output="screen",
            )
        ],
    )

    # Create the launch description and populate with arguments
    ld: LaunchDescription = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_directory_cmd)
    ld.add_action(declare_publish_robot_description_semantic_cmd)

    for node in robot_nodes:
        ld.add_action(node)
    for node in arc_lsi_nodes:
        ld.add_action(node)

    # Add actions
    ld.add_action(set_gz_initial_gui_camera_position)
    ld.add_action(declare_gz_sim_resource_path_env_var)
    ld.add_action(start_gz_cmd)
    ld.add_action(start_gz_bridge_cmd)

    return ld