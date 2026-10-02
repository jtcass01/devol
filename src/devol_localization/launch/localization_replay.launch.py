"""Offline trial: replays a recorded bag through noise injection, the filters and the scorer.

  ros2 launch devol_localization localization_replay.launch.py bag:=~/loc_bags/nominal \
      k:=1 sigma_r:=0.03 seed:=3 num_particles:=2000 output_dir:=~/loc_results/nominal/k1_s0.03_N2000/seed03

The protocol records each route once in Gazebo (localization_sim.launch.py record_bag:=...) with the
controller on ground truth, then replays it through every configuration, so every estimator sees
identical inputs and only the seeded injected noise varies between trials. The launch shuts down
two seconds after the bag ends; the scorer writes summary.json on shutdown. For a global-localization
trial replay the nominal bag with scenario:=global; for kidnapping replay a bag recorded with
scenario:=kidnap and use scenario:=kidnap here too.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

STACK_ARGS = ['scenario', 'estimators', 'k', 'sigma_r', 'seed', 'num_particles', 'output_dir', 'viz',
              'viz_headless', 'video_dir', 'viz_window', 'maze', 'posterior_times']


def launch_setup(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    share = get_package_share_directory('devol_localization')
    bag = os.path.expanduser(arg('bag'))
    if not bag:
        raise RuntimeError('bag:=<recorded bag directory> is required')

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'localization_stack.launch.py')),
        launch_arguments={**{name: arg(name) for name in STACK_ARGS}, 'use_sim_time': 'true'}.items())

    # The bag carries its own /clock, so no --clock here.
    play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, '--rate', arg('rate'),
             '--qos-profile-overrides-path', os.path.join(share, 'config', 'bag_qos_overrides.yaml'),
             '--topics', '/clock', '/tf_static', '/devol_drive/odom', '/devol_drive/sensors/lidar2d_0/scan',
             '/devol_drive/projected_map', '/devol_drive/ground_truth/odom'],
        output='screen')
    stop = RegisterEventHandler(OnProcessExit(
        target_action=play,
        on_exit=[TimerAction(period=2.0, actions=[EmitEvent(event=Shutdown(reason='bag finished'))])]))
    # Give the nodes time to start and subscribe before the first message.
    return [stack, stop, TimerAction(period=float(arg('start_delay')), actions=[play])]


def generate_launch_description():
    args = [
        ('bag', '', 'Recorded bag directory (required)'),
        ('rate', '1.0', 'Playback rate; keep 1.0 for compute-time measurements'),
        ('start_delay', '4.0', 'Seconds to wait for the nodes before playing'),
        ('scenario', 'nominal', 'nominal | global | kidnap'),
        ('estimators', 'ekf,pf', 'Comma-separated subset of ekf,pf'),
        ('k', '1.0', 'Odometry noise scale (alpha = 0.05 k)'),
        ('sigma_r', '0.03', 'Lidar range noise std, m'),
        ('seed', '0', 'Trial seed'),
        ('num_particles', '2000', 'PF particle count'),
        ('output_dir', '', 'Score the trial into this directory; empty runs no scorer'),
        ('viz', 'false', 'Open a live view per estimator'),
        ('viz_headless', 'false', 'Render the views off screen (with video_dir)'),
        ('video_dir', '', 'Write <estimator>.mp4 views here'),
        ('viz_window', '16.0', 'Side of the robot-following map view in m; 0 = whole map'),
        ('maze', 'factory', 'World the bag was recorded in (spawn pose, waypoints)'),
        ('posterior_times', '', 'Sim seconds to save PF-particles + EKF-covariance figures (needs output_dir)'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=launch_setup)])
