"""Estimators, noise injection, scoring and live views for one localization trial.

Shared by localization_sim.launch.py (live Gazebo) and localization_replay.launch.py (recorded bag);
it starts no simulator and no bag itself. Data flow:

  /devol_drive/odom, /devol_drive/sensors/lidar2d_0/scan
      -> noise_injector (k, sigma_r, seed)   -> /devol_drive/noisy/odom, /devol_drive/noisy/scan
      -> ekf_localization, pf_localization   -> /devol_drive/{ekf,pf}_pose, *_compute_time_ms
      -> localization_evaluator (vs /devol_drive/ground_truth/odom) -> output_dir/{trajectory,compute}.csv,
                                                                      summary.json
      -> localization_viz (one window per filter)

scenario sets how the filters start: nominal and kidnap start both filters at the robot's spawn
pose from the world's poses.csv; global starts the PF uniform over the free space and the EKF at a
seeded random free-space pose with a covariance spanning the map (not the centroid: in the factory
that is the spawn pose, which would start the EKF on the truth). The kidnap itself happens in the
simulator (kidnapper node, see localization_sim.launch.py), so a replayed kidnap bag needs no
extra node here.
"""

import csv
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    'scenario': ('nominal', 'nominal | global | kidnap'),
    'estimators': ('ekf,pf', 'Comma-separated subset of ekf,pf'),
    'k': ('1.0', 'Odometry noise scale: alpha1..4 = k * 0.05 (study grid 0, 1, 2, 4)'),
    'sigma_r': ('0.03', 'Lidar range noise std in m (study grid 0.01, 0.03, 0.10)'),
    'seed': ('0', 'Trial seed: injected noise and PF sampling'),
    'num_particles': ('2000', 'PF particle count (study grid 100, 500, 2000, 5000; nominal 2000)'),
    'output_dir': ('', 'Directory for the trial scores; empty runs no evaluator'),
    'viz': ('true', 'Open a live view per estimator'),
    'viz_headless': ('false', 'Render the views off screen (use with video_dir)'),
    'video_dir': ('', 'Write <estimator>.mp4 and a final <estimator>.png of each view here; empty writes none'),
    'viz_window': ('16.0', 'Side of the robot-following map view in m; 0 shows the whole map'),
    'maze': ('factory', 'World folder in devol_gazebo/worlds, for the spawn pose and waypoints'),
    'use_sim_time': ('true', 'Use /clock from Gazebo or the bag'),
    'test_case': ('0', 'Judge verification test case 1 (nominal route) or 2 (kidnapping) and print PASS/FAIL; 0 = off'),
    'finish_on_goal': ('false', 'End the whole launch once the robot has reached the last waypoint'),
    'max_duration': ('0.0', 'With finish_on_goal: end the launch after this many sim seconds anyway; 0 = never'),
    'posterior_times': ('', 'Comma list of sim seconds after start to save PF-particles + EKF-covariance figures '
                            'into output_dir; empty saves none'),
    'posterior_after_kidnap': ('1,5,15', 'With posterior_times set: also save them this many s after a kidnap'),
}


def read_poses(maze: str):
    path = os.path.join(get_package_share_directory('devol_gazebo'), 'worlds', maze, 'poses.csv')
    start, goals = (0.0, 0.0, 0.0), []
    with open(path) as f:
        for row in csv.DictReader(f):
            pose = (float(row['x']), float(row['y']), float(row['yaw']))
            if row['name'] == 'robot':
                start = pose
            else:
                goals.append((row['name'], pose))
    goals.sort(key=lambda g: g[0])
    return start, goals


def launch_setup(context):
    a = {name: context.perform_substitution(LaunchConfiguration(name)) for name in ARGS}
    for path_arg in ('output_dir', 'video_dir'):
        a[path_arg] = os.path.expanduser(a[path_arg])
    scenario = a['scenario']
    if scenario not in ('nominal', 'global', 'kidnap'):
        raise RuntimeError(f'scenario must be nominal, global or kidnap, got {scenario}')
    estimators = [e.strip() for e in a['estimators'].split(',') if e.strip()]
    use_sim_time = a['use_sim_time'].lower() == 'true'
    seed = int(a['seed'])
    start, goals = read_poses(a['maze'])
    share = get_package_share_directory('devol_localization')

    nodes = [Node(
        package='devol_localization', executable='noise_injector', name='noise_injector', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'k': float(a['k']), 'sigma_r': float(a['sigma_r']),
                     'seed': seed}])]

    if 'ekf' in estimators:
        # Seeded per trial: the factory's free-space centroid is the spawn pose, so a centroid start
        # would begin on the truth.
        ekf_init = {'init_mode': 'global', 'global_init_mean': 'random', 'global_init_seed': seed} \
            if scenario == 'global' else {
            'init_mode': 'pose', 'initial_pose': list(start)}
        nodes.append(Node(
            package='devol_localization', executable='ekf_localization', name='ekf_localization', output='screen',
            parameters=[os.path.join(share, 'config', 'ekf_localization.yaml'),
                        {'use_sim_time': use_sim_time, 'odom_topic': 'noisy/odom', 'scan_topic': 'noisy/scan',
                         'publish_tf': False, **ekf_init}]))
    if 'pf' in estimators:
        pf_init = {'init_mode': 'global'} if scenario == 'global' else {
            'init_mode': 'pose', 'initial_x': start[0], 'initial_y': start[1], 'initial_yaw': start[2]}
        nodes.append(Node(
            package='devol_localization', executable='pf_localization', name='pf_localization', output='screen',
            parameters=[os.path.join(share, 'config', 'pf_localization.yaml'),
                        {'use_sim_time': use_sim_time, 'odom_topic': '/devol_drive/noisy/odom',
                         'scan_topic': '/devol_drive/noisy/scan', 'publish_tf': False,
                         'num_particles': int(a['num_particles']), 'seed': seed,
                         'particles_publish_count': 0, **pf_init}]))

    if a['output_dir']:
        config = {'k': float(a['k']), 'sigma_r': float(a['sigma_r']), 'seed': seed,
                  'num_particles': int(a['num_particles']), 'estimators': estimators, 'maze': a['maze'],
                  'waypoint_names': [g[0] for g in goals]}
        finish = a['finish_on_goal'].lower() == 'true'
        evaluator = Node(
            package='devol_localization', executable='localization_evaluator', name='localization_evaluator',
            output='screen', emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time, 'output_dir': a['output_dir'], 'scenario': scenario,
                         'estimators': estimators, 'start_pose': list(start),
                         'waypoints': [c for _, g in goals for c in g[:2]],
                         'config_json': json.dumps(config), 'test_case': int(a['test_case']),
                         'finish_on_goal': finish, 'max_duration': float(a['max_duration']),
                         # A live test case starts a fresh sim at t = 0; a later first stamp is a stale sim.
                         'max_start_time': 10.0 if int(a['test_case']) else 0.0}])
        nodes.append(evaluator)
        if finish:
            # The scorer exits when the route is done; take everything else down with it.
            nodes.append(RegisterEventHandler(OnProcessExit(
                target_action=evaluator,
                on_exit=[EmitEvent(event=Shutdown(reason='localization trial finished'))])))

    if a['posterior_times'] and a['output_dir']:
        def floats(text):
            return [float(v) for v in text.split(',') if v.strip()]
        nodes.append(Node(
            package='devol_localization', executable='posterior_snapshot', name='posterior_snapshot',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'output_dir': a['output_dir'],
                         'times': floats(a['posterior_times']),
                         'after_kidnap': floats(a['posterior_after_kidnap']) or [1.0],
                         'title': f'Posteriors ({scenario}, N={a["num_particles"]}):'}]))

    if a['viz'].lower() == 'true':
        for est in estimators:
            video = os.path.join(a['video_dir'], f'{est}.mp4') if a['video_dir'] else ''
            snapshot = os.path.join(a['video_dir'], f'{est}.png') if a['video_dir'] else ''
            nodes.append(Node(
                package='devol_localization', executable='localization_viz', name=f'{est}_viz', output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'mode': est, 'video_file': video, 'snapshot_file': snapshot,
                             'headless': a['viz_headless'].lower() == 'true', 'window': float(a['viz_window']),
                             'title': f'{est.upper()} vs Gazebo ground truth ({scenario}, k={a["k"]}, '
                                      f'sigma_r={a["sigma_r"]} m, seed {seed}'
                                      + (f', N={a["num_particles"]})' if est == 'pf' else ')')}]))
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, (default, desc) in ARGS.items()]
        + [OpaqueFunction(function=launch_setup)])
