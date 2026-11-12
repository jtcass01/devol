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

LOC_PKG: str = 'devol_localization'


def launch_setup(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    def check(argument: str) -> bool:
        return arg(argument).lower() == 'true'

    actions = []

    sim_share = get_package_share_directory('devol_sim')
    seed: int = int(arg('seed'))
    odom_topic = '/devol_drive/noisy/odom' if check('noise') else '/devol_drive/odom'

    # Sim
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

    # Estimators
    actions += [
        Node(
            package=LOC_PKG,
            executable='ekf_node',
            name='ekf_localization_node',
            output='screen',
            parameters=[
                {'use_sim_time': True, 'odom_topic': odom_topic, 'seed': int(arg('seed'))}
            ],
        ),
        Node(
            package=LOC_PKG,
            executable='pf_node',
            name='particle_filter_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'seed': int(arg('seed')),
                    'odom_topic': odom_topic,
                    'num_particles': int(arg('pf_num_particles')),
                }
            ],
        ),
        Node(
            package=LOC_PKG,
            executable='dr_node',
            name='dead_reckoning_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'seed': int(arg('seed')),
                    'odom_topic': odom_topic,
                }
            ],
        ),
    ]

    # visualization
    if check('show_vis'):
        # Start Visualization Node
        actions.append(
            Node(
                package=LOC_PKG,
                executable='viz_node',
                name='localization_vis_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        )

    # log metrics
    if check('log_metrics'):
        actions.append(
            Node(
                package=LOC_PKG,
                executable='localization_logging_node',
                name='localization_logging_node',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': True,
                        'rate_hz': float(arg('log_rate_hz')),
                        'results_dir': arg('results_dir'),
                    }
                ],
            )
        )

    # Kidnapping
    if check('kidnap'):
        actions.append(
            Node(
                package=LOC_PKG,
                executable='kidnapper_node',
                name='kidnapper_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        )

    # Noise Injection
    if check('noise'):
        actions.append(
            Node(
                package=LOC_PKG,
                executable='noise_injector_node',
                name='noise_injector_node',
                output='screen',
                parameters=[
                    {'use_sim_time': True, 'seed': arg('seed'), 'k': float(arg('noise_scale'))}
                ],
            )
        )

    # Shutdown on goal
    if check('monitor_goal'):
        goal_monitor = Node(
            package=LOC_PKG,
            executable='goal_monitor_node',
            name='goal_monitor_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )

        actions += [
            goal_monitor,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=goal_monitor,
                    on_exit=[EmitEvent(event=Shutdown(reason='Goal reached. Test complete.'))],
                )
            ),
        ]

    # Planner
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
            'planner': 'rrt_star',
            'max_linear_velocity': arg('max_linear_velocity'),
            'x': '0.0',
            'y': '0.0',
            'yaw': '0.0',
        }.items(),
    )

    wait_for_ekf = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'echo',
            '--once',
            '/devol_drive/ekf_pose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
        ],
        output='log',
    )

    actions += [
        wait_for_ekf,
        RegisterEventHandler(OnProcessExit(target_action=wait_for_ekf, on_exit=[planner])),
    ]

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
        ('log_rate_hz', '10.0', 'Max logging rate in Hz.'),
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
