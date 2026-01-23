from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument('name', default_value='devol',
                          description='Prefix for all joint names'),
]


def generate_launch_description():
    # Define file names
    urdf_package = "devol_drive_description"
    urdf_filename = "devol_drive.urdf.xacro"
    rviz_config_filename = "devol_drive.rviz"

    # Define paths
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_path = PathJoinSubstitution(
        [pkg_share_description, "urdf", urdf_filename]
    )
    default_rviz_path = PathJoinSubstitution(
        [pkg_share_description, "rviz", rviz_config_filename]
    )

    # Launch configuration variables
    jsp_gui = LaunchConfiguration("jsp_gui")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    urdf_model = LaunchConfiguration("urdf_model")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare launch arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        "jsp_gui",
        default_value="true",
        choices=["true", "false"],
        description="Launch the Joint State Publisher GUI",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_path,
        description="Path to RViz config file",
    )
    declare_urdf_model_cmd = DeclareLaunchArgument(
        "urdf_model",
        default_value=default_urdf_path,
        description="Path to the URDF model file",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch RViz with the robot description",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use Gazebo simulation clock",
    )

    robot_description_content: ParameterValue = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'parent:=world '
    ]), value_type=str)

    # Subscribe to the joint states of the robot, and publish them to the robot state publisher
    start_robot_state_publisher_cmd: Node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='a200_0000',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description_content}])

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd: Node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='a200_0000',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(jsp_gui),
    )

    start_joint_state_publisher_gui_cmd: Node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace='a200_0000',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui),
    )

    # Launch RViz
    start_rviz_cmd: Node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Create the launch description and populate with arguments
    ld = LaunchDescription(ARGUMENTS)

    # Declare the launch options
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_urdf_model_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld