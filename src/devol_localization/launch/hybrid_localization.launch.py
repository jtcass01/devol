"""Hybrid EKF + PF estimator: a second EKF instance plus the supervisor that re-seeds it from the PF.

Needs the particle filter node running already (pf_localization.launch.py or
localization_stack.launch.py); it reads that PF's pose and does not change it. The standalone
EKF (ekf_pose) is not touched either: the hybrid EKF publishes /devol_drive/hybrid_pose and
listens for re-seeds on /devol_drive/hybrid_initialpose, so both can run in the same trial.

Example, next to the study stack (which feeds the filters noisy/odom and noisy/scan):
  ros2 launch devol_localization hybrid_localization.launch.py odom_topic:=noisy/odom scan_topic:=noisy/scan
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    'use_sim_time': ('true', 'Use /clock from Gazebo or the bag'),
    'odom_topic': (
        'odom',
        'Odometry for the hybrid EKF, relative to /devol_drive (the study stack uses noisy/odom)',
    ),
    'scan_topic': (
        'sensors/lidar2d_0/scan',
        'Scan for the hybrid EKF and the supervisor, relative to /devol_drive '
        '(the study stack uses noisy/scan)',
    ),
    'pf_pose_topic': ('/devol_drive/pf_pose', 'Particle filter pose'),
    'initial_pose': (
        '',
        'x,y,yaw to start the hybrid EKF at; empty starts it from TF like the EKF node',
    ),
}


def launch_setup(context):
    a = {name: LaunchConfiguration(name).perform(context) for name in ARGS}
    use_sim_time = a['use_sim_time'].lower() == 'true'
    share = get_package_share_directory('devol_localization')
    with open(os.path.join(share, 'config', 'ekf_localization.yaml')) as f:
        ekf_params = yaml.safe_load(f)['ekf_localization']['ros__parameters']
    ekf_params.update(
        {
            'use_sim_time': use_sim_time,
            'odom_topic': a['odom_topic'],
            'scan_topic': a['scan_topic'],
            'pose_topic': 'hybrid_pose',
            'compute_time_topic': 'hybrid_compute_time_ms',
            'initial_pose_topic': 'hybrid_initialpose',
            'publish_tf': False,
        }
    )
    if a['initial_pose']:
        ekf_params.update(
            {'init_mode': 'pose', 'initial_pose': [float(v) for v in a['initial_pose'].split(',')]}
        )
    scan = (
        a['scan_topic'] if a['scan_topic'].startswith('/') else '/devol_drive/' + a['scan_topic']
    )
    return [
        Node(
            package='devol_localization',
            executable='ekf_localization',
            name='hybrid_ekf',
            output='screen',
            parameters=[ekf_params],
        ),
        Node(
            package='devol_localization',
            executable='hybrid_supervisor',
            name='hybrid_supervisor',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'ekf_pose_topic': '/devol_drive/hybrid_pose',
                    'pf_pose_topic': a['pf_pose_topic'],
                    'scan_topic': scan,
                    'reset_topic': '/devol_drive/hybrid_initialpose',
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, (d, h) in ARGS.items()]
        + [OpaqueFunction(function=launch_setup)]
    )
