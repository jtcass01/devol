import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    sim_share = get_package_share_directory('devol_sim')
    loc_share = 'devol_localization' 
    namespace = '/devol_drive'
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
                'map_odom_tf': 'none',
                'gz_gui': arg('gz_gui'),
                'rviz': 'false',
            }.items(),
        )
    )

    # Bridge Ground Truth from Gazebo
    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ground_truth_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[f'{namespace}/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
            remappings=[
                (f'{namespace}/ground_truth/odom', f'{namespace}/ground_truth/odom')
            ]
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
            parameters=[{'use_sim_time': True}],
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
            remappings=[
                ('/devol_drive/odom', '/devol_drive/ground_truth/odom')
            ]
        )
    )

    return actions

def generate_launch_description():
    args = [
        ('maze', 'factory', 'World to load'),
        ('planner', 'rrt_star', 'a_star | rrt | rrt_star'),
        ('max_linear_velocity', '1.5', 'Path follower top speed in m/s'),
        ('gz_gui', 'false', 'Show the Gazebo GUI'),
    ]
    
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=launch_setup)]
    )