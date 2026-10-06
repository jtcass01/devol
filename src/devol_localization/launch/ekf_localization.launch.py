import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('devol_localization'), 'config', 'ekf_localization.yaml'
    )

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Publish map -> odom (remove the static one first).',
    )

    ekf = Node(
        package='devol_localization',
        executable='ekf_localization',
        name='ekf_localization',
        output='screen',
        parameters=[
            params,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            },
        ],
    )

    return LaunchDescription([use_sim_time, publish_tf, ekf])
