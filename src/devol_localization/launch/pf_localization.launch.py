from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = join(
        get_package_share_directory('devol_localization'), 'config', 'pf_localization.yaml'
    )

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params, description='Particle filter parameter file'
    )
    declare_num_particles = DeclareLaunchArgument(
        'num_particles', default_value='500', description='Number of particles'
    )
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf', default_value='false', description='Broadcast map -> odom'
    )
    declare_init_mode = DeclareLaunchArgument(
        'init_mode', default_value='tf', description='tf | pose | global'
    )

    pf_node = Node(
        package='devol_localization',
        executable='pf_localization',
        name='pf_localization',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'num_particles': LaunchConfiguration('num_particles'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'init_mode': LaunchConfiguration('init_mode'),
            },
        ],
    )

    return LaunchDescription(
        [declare_params, declare_num_particles, declare_publish_tf, declare_init_mode, pf_node]
    )
