import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    actions = []

    sim_share = get_package_share_directory('devol_sim')
    loc_share = 'devol_localization'
    noisy: bool = arg('noise').lower() == 'true'
    odom_topic = '/devol_drive/noisy/odom' if noisy else '/devol_drive/odom'

    # Start the Simulation Pipeline
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_share, 'launch', 'motion_planner_sim.launch.py')
            ),
            launch_arguments={
                'maze': arg('maze'),
                'planner': 'none',
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
            parameters=[{'use_sim_time': True,
                         'odom_topic': odom_topic,
                         'seed': arg('seed')}],
        )
    )

    # Start Particle Filter
    actions.append(
        Node(
            package=loc_share,
            executable='pf_node',
            name='particle_filter_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'seed': arg('seed'),
                    'odom_topic': odom_topic,
                    'num_particles': int(arg('pf_num_particles')),
                }
            ],
        )
    )

    # Start Dead Reckoning
    actions.append(
        Node(
            package=loc_share,
            executable='dr_node',
            name='dead_reckoning_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'seed': arg('seed'),
                    'odom_topic': odom_topic,
                }
            ],
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
            parameters=[{'use_sim_time': True, 'results_dir': arg('results_dir')}],
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

    actions.append(
        Node(
            package=loc_share,
            executable='noise_injector_node',
            name='noise_injector_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True, 
                    'seed': arg('seed'),
                    'k': float(arg('noise_scale'))
                }
            ],
            condition=IfCondition(LaunchConfiguration('noise')),
        )
    )

    goal_monitor = Node(
        package=loc_share,
        executable='goal_monitor_node',
        name='goal_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('monitor_goal')),
    )

    actions.append(goal_monitor)

    planner: Node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('devol_local_planner'),
                'launch',
                'local_planner.launch.py',
            )
        ),
        launch_arguments={
            'namespace': '/devol_drive',
            'planner': "rrt_star",
            'max_linear_velocity': arg('max_linear_velocity'),
            'x': '0.0',
            'y': '0.0',
            'yaw': '0.0',
        }.items(),
    )

    wait_for_ekf = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', 
             '/devol_drive/ekf_pose',
             'geometry_msgs/msg/PoseWithCovarianceStamped',],
        output='log',
    )

    actions.append(wait_for_ekf)

    actions.append(
        RegisterEventHandler(OnProcessExit(target_action=wait_for_ekf, on_exit=[planner]))
    )

    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=goal_monitor,
                on_exit=[EmitEvent(event=Shutdown(reason='Goal reached. Test complete.'))],
            )
        )
    )

    return actions


def generate_launch_description():
    args = [
        ('maze', 'factory', 'World to load'),
        ('planner', 'rrt_star', 'a_star | rrt | rrt_star'),
        ('max_linear_velocity', '1.0', 'Path follower top speed in m/s'),
        ('pf_num_particles', '400', 'Number of Particles for Particle Filter'),
        ('gz_gui', 'false', 'Show the Gazebo GUI'),
        ('kidnap', 'true', 'kidnap?'),
        ('monitor_goal', 'true', 'Monitor goal for shutdown?'),
        ('log_metrics', 'true', 'log metrics?'),
        ('show_vis', 'true', 'Visualize?'),
        ('seed', '31', 'Seed for RNG'),
        ('results_dir', 'results', 'relative directory for results'),
        ('noise', 'true', 'Inject odometry noise?'),
        ('noise_scale', '0.0', 'scale of odometry noise injection'),
    ]

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=launch_setup)]
    )
