"""Live Gazebo run of the localization study: sim + ground truth + filters + live views (+ bag, kidnap).

  ros2 launch devol_localization localization_sim.launch.py                       # nominal, EKF + PF windows
  ros2 launch devol_localization localization_sim.launch.py scenario:=kidnap      # teleport 10 s after Goal 1
  ros2 launch devol_localization localization_sim.launch.py scenario:=global
  ros2 launch devol_localization localization_sim.launch.py record_bag:=~/loc_bags/nominal filters:=false

What it adds to devol_sim's motion_planner_sim.launch.py:
- a bridge for /devol_drive/ground_truth/odom (gz OdometryPublisher on the robot, world frame = map);
- ground_truth_tf, which publishes map -> odom from ground truth so the planner and controller drive on
  the true pose (the protocol: estimator error never changes the trajectory). The sim is started with
  map_odom_tf:=none so the static map -> odom placeholder is not published as well;
- kidnapper when scenario:=kidnap: teleports the robot to kidnap_target kidnap_time s after it reaches
  Goal 1 (kidnap_trigger:=time counts from the start instead; that can fire before Goal 1);
- ros2 bag record of the raw streams (record_bag), for replaying every study configuration offline;
- localization_stack.launch.py (filters, noise, scorer, views) unless filters:=false.

Depends on the sim fixes still being finished on the WSL machine: odom -> a200_base_link on /tf
(bridged /model/devol_drive/tf plus the a200_base_link alias), so the planner's map -> base lookup
resolves through ground_truth_tf.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STACK_ARGS = ['scenario', 'estimators', 'k', 'sigma_r', 'seed', 'num_particles', 'output_dir', 'viz',
              'viz_headless', 'video_dir', 'viz_window', 'maze', 'test_case', 'finish_on_goal', 'max_duration',
              'posterior_times', 'posterior_after_kidnap']

RECORD_TOPICS = [
    '/clock',
    '/tf_static',
    '/devol_drive/odom',
    '/devol_drive/sensors/lidar2d_0/scan',
    '/devol_drive/projected_map',
    '/devol_drive/ground_truth/odom',
    '/devol_drive/kidnap_event',
]


def launch_setup(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    sim_share = get_package_share_directory('devol_sim')
    share = get_package_share_directory('devol_localization')
    namespace = '/devol_drive'
    actions = []

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'motion_planner_sim.launch.py')),
        launch_arguments={'maze': arg('maze'), 'planner': arg('planner'), 'map_odom_tf': 'none',
                          # Honored by sim launch files that declare them; ignored otherwise.
                          'gz_gui': arg('gz_gui'), 'rviz': 'false'}.items()))

    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ground_truth_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[f'{namespace}/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry']))
    actions.append(Node(
        package='devol_localization', executable='ground_truth_tf', name='ground_truth_tf', output='screen',
        parameters=[{'use_sim_time': True}]))

    if arg('scenario') == 'kidnap':
        target = [float(v) for v in arg('kidnap_target').split(',')]
        actions.append(Node(
            package='devol_localization', executable='kidnapper', name='kidnapper', output='screen',
            parameters=[{'use_sim_time': True, 'kidnap_time': float(arg('kidnap_time')), 'target': target,
                         'trigger': arg('kidnap_trigger')}]))

    bag = arg('record_bag')
    if bag:
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.expanduser(bag), '--use-sim-time',
                 '--qos-profile-overrides-path', os.path.join(share, 'config', 'bag_qos_overrides.yaml'),
                 '--topics', *RECORD_TOPICS],   # Lyrical's rosbag2 takes topics only after --topics
            output='screen',
            # On Ctrl-C launch escalates SIGINT -> SIGTERM -> SIGKILL after 5 s each by default, which can kill
            # the recorder before it writes metadata.yaml. Give it time to close the bag.
            sigterm_timeout='30', sigkill_timeout='30'))

    if arg('filters').lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'localization_stack.launch.py')),
            launch_arguments={name: arg(name) for name in STACK_ARGS}.items()))
    return actions


def generate_launch_description():
    args = [
        ('maze', 'factory', 'World to load'),
        ('planner', 'rrt_star', 'a_star | rrt | rrt_star'),
        ('gz_gui', 'false', 'Show the Gazebo GUI (if the sim launch supports it)'),
        ('filters', 'true', 'Run the estimators, scorer and views live'),
        ('record_bag', '', 'Record the raw streams to this bag directory for offline replay'),
        ('kidnap_trigger', 'after_waypoint', 'scenario:=kidnap: after_waypoint (kidnap_time s after reaching '
                                             'Goal 1) or time (kidnap_time s after start)'),
        ('kidnap_time', '10.0', 'scenario:=kidnap: delay in sim seconds before the teleport'),
        ('kidnap_target', '5.45,2.03,0.0', 'scenario:=kidnap: x,y,yaw to teleport to (default Goal 2)'),
        ('scenario', 'nominal', 'nominal | global | kidnap'),
        ('estimators', 'ekf,pf', 'Comma-separated subset of ekf,pf,hybrid (hybrid needs pf)'),
        ('k', '1.0', 'Odometry noise scale (alpha = 0.05 k)'),
        ('sigma_r', '0.03', 'Lidar range noise std, m'),
        ('seed', '0', 'Trial seed'),
        ('num_particles', '2000', 'PF particle count'),
        ('output_dir', '', 'Score the run into this directory; empty runs no scorer'),
        ('viz', 'true', 'Open a live view per estimator'),
        ('viz_headless', 'false', 'Render the views off screen (with video_dir)'),
        ('video_dir', '', 'Write <estimator>.mp4 views here'),
        ('viz_window', '16.0', 'Side of the robot-following map view in m; 0 = whole map'),
        ('test_case', '0', 'Judge test case 1 or 2 and print PASS/FAIL (needs output_dir); 0 = off'),
        ('finish_on_goal', 'false', 'End the launch once the robot reaches the last waypoint (needs output_dir)'),
        ('max_duration', '0.0', 'With finish_on_goal: end after this many sim seconds anyway; 0 = never'),
        ('posterior_times', '', 'Sim seconds to save PF-particles + EKF-covariance figures (needs output_dir)'),
        ('posterior_after_kidnap', '1,5,15', 'Also save them this many s after a kidnap'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=launch_setup)])
