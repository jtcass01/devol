from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from tempfile import gettempdir
from os.path import join

ARGUMENTS = [
    DeclareLaunchArgument('name', default_value='devol',
                          description=''),
]


def generate_launch_description():
    # Define file names
    devol_description_package_name: str = "devol_description"
    devol_gz_package_name: str = "devol_gazebo"
    devol_gazebo_launch_filename: str = "devol_gazebo.launch.py"
    gz_package_name: str = "ros_gz_sim"
    gz_launch_filename: str = "gz_sim.launch.py"
    devol_urdf_filename: str = "devol.urdf.xacro"
    devol_rviz_config_filename: str = "view_devol.rviz"

    # Define paths
    devol_description_path: FindPackageShare = FindPackageShare(devol_description_package_name)
    devol_urdf_path: PathJoinSubstitution = PathJoinSubstitution(
        [devol_description_path, "urdf", devol_urdf_filename]
    )
    devol_rviz_path: PathJoinSubstitution = PathJoinSubstitution(
        [devol_description_path, "rviz", devol_rviz_config_filename]
    )

    devol_gz_path: FindPackageShare = FindPackageShare(devol_gz_package_name)
    devol_launch_path: PathJoinSubstitution = PathJoinSubstitution(
        [devol_gz_path, "launch", devol_gazebo_launch_filename]
    )
    devol_gz_model_directory: PathJoinSubstitution = PathJoinSubstitution(
        [devol_gz_path, "worlds"]
    )

    gz_path: FindPackageShare = FindPackageShare(gz_package_name)
    gz_launch_path: PathJoinSubstitution = PathJoinSubstitution(
        [gz_path, "launch", gz_launch_filename]
    )

    tmp_sdf_path: str = join(gettempdir(), "devol.sdf")

    # Launch configuration variables
    jsp_gui = LaunchConfiguration("jsp_gui")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    urdf_model = LaunchConfiguration("urdf_model")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_directory = LaunchConfiguration("world")

    # Declare launch arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        "jsp_gui",
        default_value="true",
        choices=["true", "false"],
        description="Launch the Joint State Publisher GUI",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=devol_rviz_path,
        description="Path to RViz config file",
    )
    declare_urdf_model_cmd = DeclareLaunchArgument(
        "urdf_model",
        default_value=devol_urdf_path,
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
        default_value="true",
        choices=["true", "false"],
        description="Use Gazebo simulation clock",
    )

    declare_world_directory_cmd = DeclareLaunchArgument(
        "world",
        default_value="factory",
        choices=["devol", "factory"]
    )

    robot_description_content: ParameterValue = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'name:=', LaunchConfiguration('name'), ' ',
    ]), value_type=str)

    # Subscribe to the joint states of the robot, and publish them to the robot state publisher
    start_robot_state_publisher_cmd: Node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description_content}])

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

    # Launch Gazebo
    start_gz_cmd: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': PathJoinSubstitution(
                [devol_gz_model_directory, world_directory, "model.sdf"]
            )
        }.items(),
    )

    declare_gz_sim_resource_path_env_var = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution(
            [devol_gz_model_directory, world_directory, 
             ":", FindPackageShare("ur_description"), 
             ":", FindPackageShare("robotiq_description"), 
             ":", FindPackageShare("devol_description"), 
             ":$GZ_SIM_RESOURCE_PATH"])
    )

    start_gz_bridge_cmd: Node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution(
                [devol_gz_path, 'config', 'bridge.yaml']
            ),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Create the launch description and populate with arguments
    ld = LaunchDescription(ARGUMENTS)

    # Declare the launch options
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_urdf_model_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_directory_cmd)

    # Add actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(declare_gz_sim_resource_path_env_var)
    ld.add_action(start_gz_cmd)
    ld.add_action(start_gz_bridge_cmd)

    return ld