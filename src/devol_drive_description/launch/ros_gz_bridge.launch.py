from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    a200_urdf_package = "a200_description"
    a200_urdf_pkg_share = get_package_share_directory(a200_urdf_package)

    devol_urdf_package = "devol_description"
    devol_urdf_pkg_share = get_package_share_directory(devol_urdf_package)

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='/devol_drive',
        description='Namespace for topics'
    )

    def launch_setup(context):
        namespace = context.perform_substitution(LaunchConfiguration('namespace'))

        a200_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(a200_urdf_pkg_share, 'launch', 'ros_gz_bridge.launch.py')
            ),
            launch_arguments={
                'namespace': namespace
            }.items()
        )

        devol_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(devol_urdf_pkg_share, 'launch', 'ros_gz_bridge.launch.py')
            ),
            launch_arguments={
                'namespace': namespace
            }.items()
        )

        return [a200_bridge, devol_bridge]

    return LaunchDescription([declare_namespace, OpaqueFunction(function=launch_setup)])
