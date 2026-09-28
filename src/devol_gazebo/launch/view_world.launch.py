from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define file names
    project_gz_package_name: str = "devol_gazebo"
    gz_package_name: str = "ros_gz_sim"
    gz_launch_filename: str = "gz_sim.launch.py"

    # Define paths
    project_gz_path: FindPackageShare = FindPackageShare(
        project_gz_package_name
    )
    project_world_directory: PathJoinSubstitution = PathJoinSubstitution(
        [project_gz_path, "worlds"]
    )
    gz_path: FindPackageShare = FindPackageShare(gz_package_name)
    gz_launch_path: PathJoinSubstitution = PathJoinSubstitution(
        [gz_path, "launch", gz_launch_filename]
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_directory = LaunchConfiguration("world")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use Gazebo simulation clock",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="DEBUG",
        choices=["DEBUG"],
        description="",
    )
    declare_world_directory_cmd = DeclareLaunchArgument(
        "world",
        default_value="moon_terrain",
        choices=["empty", "factory", "moon_terrain"],
    )

    start_gz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            "gz_args": [
                TextSubstitution(text="-r -v 4 "),
                PathJoinSubstitution(
                    [project_world_directory, world_directory, "maze_world.sdf"]
                ),
            ]
        }.items(),
    )

    declare_gz_sim_resource_path_env_var = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution(
            [
                project_world_directory,
                world_directory,
                ":",
                FindPackageShare("ur_description"),
                ":",
                FindPackageShare("ars_description"),
                ":",
                FindPackageShare("ars_ee_description"),
                ":",
                FindPackageShare("ars_stand_description"),
                ":",
                FindPackageShare("tlt_part_description"),
                ":$GZ_SIM_RESOURCE_PATH",
            ]
        ),
    )

    start_gz_bridge_cmd: Node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # Create the launch description and populate with arguments
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_world_directory_cmd)

    # Add actions
    ld.add_action(declare_gz_sim_resource_path_env_var)
    ld.add_action(start_gz_cmd)
    ld.add_action(start_gz_bridge_cmd)

    return ld
