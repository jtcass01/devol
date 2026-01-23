from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    a200_urdf_package = "a200_description"
    a200_urdf_pkg_share = get_package_share_directory(a200_urdf_package)
   
    a200_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(a200_urdf_pkg_share, 'launch', 'ros_gz_bridge.launch.py')
        )
    )

    return LaunchDescription([a200_bridge])
