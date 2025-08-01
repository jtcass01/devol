from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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

    # Launch configuration variables
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_model = LaunchConfiguration("urdf_model")
    name = LaunchConfiguration("name")
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
    declare_urdf_model_cmd = DeclareLaunchArgument(
        "urdf_model",
        default_value=default_urdf_path,
        description="Path to the URDF model file",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use Gazebo simulation clock",
    )
    declare_warehouse_sqlite_path_cmd = DeclareLaunchArgument(
        "warehouse_sqlite_path",
        default_value=PathJoinSubstitution(
            [pkg_share_description, "config", "warehouse_ros.sqlite"]
        ),
        description="Path to the SQLite database for the warehouse",
    )
    declare_publish_robot_description_semantic_cmd = DeclareLaunchArgument(
        "publish_robot_description_semantic",
        default_value="true",
        choices=["true", "false"],
        description="Publish the robot description semantic",
    )

    robot_description_content: ParameterValue = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'name:=', name, ' ',
        'use_gazebo:=false', ' ',
        'use_mock_hardware:=true'
    ]), value_type=str)

    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(moveit_package),
            "config",
            "ros2_controllers.yaml"
        ]
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="devol", package_name=moveit_package)
        .to_moveit_configs()
    )

    # Subscribe to the joint states of the robot, and publish them to the robot state publisher
    start_robot_state_publisher_cmd: Node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description])

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

    start_controller_manager_cmd: Node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both"
    )

    start_robot_state_broadcaster_cmd: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    # Spawn UR controller
    spawn_ur_controller_cmd: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_manipulator_controller", "--param-file", robot_controllers],
    )

    # Spawn UR controller
    spawn_hand_controller_cmd: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--param-file", robot_controllers],
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
            moveit_config.robot_description_semantic,
            moveit_config.robot_description,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time}],
    )

    # Create the launch description and populate with arguments
    ld = LaunchDescription(ARGUMENTS)

    # Declare the launch options
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_urdf_model_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_warehouse_sqlite_path_cmd)
    ld.add_action(declare_publish_robot_description_semantic_cmd)

    # Add actions
    ld.add_action(start_controller_manager_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_move_group_cmd)
    ld.add_action(spawn_ur_controller_cmd)
    ld.add_action(spawn_hand_controller_cmd)
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_ur_controller_cmd,
            on_exit=[start_robot_state_broadcaster_cmd]
        )
    ))
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_robot_state_broadcaster_cmd,
            on_exit=[start_rviz_cmd]
        )
    ))

    return ld