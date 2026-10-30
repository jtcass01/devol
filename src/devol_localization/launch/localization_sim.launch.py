import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    sim_share = get_package_share_directory('devol_sim')
    loc_share = 'devol_localization'
    actions = []

    # Start the Simulation Pipeline
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_share, 'launch', 'motion_planner_sim.launch.py')
            ),
            launch_arguments={
                'maze': arg('maze'),
                'planner': arg('planner'),
                'max_linear_velocity': arg('max_linear_velocity'),
                'gz_gui': arg('gz_gui'),
                'rviz': 'false',
                'map_odom_tf': 'none',
            }.items(),
        )
    )

    # Start Extended Kalman Filter
    actions.append(
        Node(
            package=loc_share,
            executable='ekf_node',
            name='ekf_localization_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    )

    # Start Particle Filter
    actions.append(
        Node(
            package=loc_share,
            executable='pf_node',
            name='particle_filter_node',
            output='screen',
            parameters=[{'use_sim_time': True, 'num_particles': arg('pf_num_particles')}],
        )
    )

    # Start Dead Reckoning
    actions.append(
        Node(
            package=loc_share,
            executable='dr_node',
            name='dead_reckoning_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    )

    # Start Visualization Node
    actions.append(
        Node(
            package=loc_share,
            executable='viz_node',
            name='localization_vis_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('show_vis')),
        )
    )

    # Start Metrics Node
    actions.append(
        Node(
            package=loc_share,
            executable='metrics_node',
            name='metrics_vis_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('log_metrics')),
        )
    )

    actions.append(
        Node(
            package=loc_share,
            executable='kidnapper_node',
            name='kidnapper_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('kidnap')),
        )
    )

    return actions

def generate_launch_description():
    args = [
        ('maze', 'factory', 'World to load'),
        ('planner', 'rrt_star', 'a_star | rrt | rrt_star'),
        ('max_linear_velocity', '1.0', 'Path follower top speed in m/s'),
        ('pf_num_particles', '400', 'Number of Particles for Particle Filter'),
        ('gz_gui', 'true', 'Show the Gazebo GUI'),
        ('kidnap', 'true', 'kidnap?'),
        ('log_metrics', 'true', 'log metrics?'),
        ('show_vis', 'true', 'Visualize?'),
    ]

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=launch_setup)]
    )
