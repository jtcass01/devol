"""Verification test cases for the localization study, preconfigured: watch them run and read PASS/FAIL.

  ros2 launch devol_localization localization_test_cases.launch.py test_case:=1   # nominal route
  ros2 launch devol_localization localization_test_cases.launch.py test_case:=2   # kidnapping

Both start the factory sim headless with the controller on ground truth, the EKF and the PF at the
nominal study point (N = 2000, k = 1, sigma_r = 0.03 m, seed 0) and a live window per filter. The run
ends by itself a few seconds after the robot reaches Goal 3 (or FAILs at max_duration), then prints the
verdict and leaves results, the two view videos, a final screenshot of each, and the intermediate
posterior figures (PF particles + EKF 2-sigma ellipse at the same instant) in output_dir.

Test case 1, nominal three-waypoint route: expected output is the Goal 1-3 poses in
devol_gazebo/worlds/factory/poses.csv. PASS when the EKF and the PF are both within 0.25 m of ground
truth at every waypoint and dead reckoning is worse than both.

Test case 2, kidnapping: kidnap_delay s after the robot reaches Goal 1, it is teleported onto Goal 2
(5.45, 2.03, 0) without the estimators being told; the planner then drives on to Goal 3. Expected
output is that pose. PASS when the PF returns within 0.25 m and stays there before the 60 s recovery
timeout; the EKF's outcome is reported either way.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def is_sim_process(argv):
    """True for a Gazebo server (`gz sim ...`, run directly or through ruby) or a ros_gz parameter_bridge.

    Matches on the program and its first arguments, not on the whole command line, so a shell whose
    script merely mentions `gz sim` (or pkill -f "gz sim") is not mistaken for a running sim.
    """
    args = list(argv)
    if not args:
        return False
    program = os.path.basename(args[0])
    if program == 'parameter_bridge' or program.startswith('gz-sim'):
        return True
    if program.startswith('ruby'):
        args = args[1:]
    return len(args) >= 2 and os.path.basename(args[0]) == 'gz' and args[1] == 'sim'


def running_sim_processes(proc='/proc'):
    """Command lines of Gazebo servers and ros_gz bridges already running ([] where /proc is unavailable)."""
    found = []
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit() and int(p) != os.getpid()]
    except OSError:
        return []
    for pid in pids:
        try:
            with open(os.path.join(proc, pid, 'cmdline'), 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        if is_sim_process(argv):
            found.append(f'{pid} {" ".join(argv)}')
    return found


def launch_setup(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    case = arg('test_case')
    if case not in ('1', '2'):
        raise RuntimeError(f'test_case must be 1 or 2, got {case}')
    if arg('check_running_sim') == 'true':
        stale = running_sim_processes()
        if stale:
            raise RuntimeError(
                'A Gazebo sim or ros_gz bridge from an earlier run is still running, and its clock and ground '
                'truth would be scored as this run:\n  ' + '\n  '.join(stale) +
                '\nWait for it to exit or stop it (pkill -f "gz sim"; pkill -f parameter_bridge), then relaunch. '
                '(check_running_sim:=false skips this check.)')
    out = os.path.expanduser(arg('output_dir') or f'~/loc_results/test_case_{case}')
    sim_args = {
        'scenario': 'nominal' if case == '1' else 'kidnap',
        'kidnap_trigger': 'after_waypoint',
        'kidnap_time': arg('kidnap_delay'),
        'kidnap_target': '5.45,2.03,0.0',
        'estimators': 'ekf,pf',
        'k': '1.0',
        'sigma_r': '0.03',
        'num_particles': arg('num_particles'),
        'seed': arg('seed'),
        'output_dir': out,
        'video_dir': out if arg('record_video') == 'true' else '',
        'viz': arg('viz'),
        'viz_window': arg('viz_window'),
        'gz_gui': arg('gz_gui'),
        'test_case': case,
        'finish_on_goal': 'true',
        'max_duration': arg('max_duration'),
        'posterior_times': arg('posterior_times'),
        'posterior_after_kidnap': '1,5,15',
        'record_bag': os.path.join(out, 'bag') if arg('record_bag') == 'true' else '',
    }
    share = get_package_share_directory('devol_localization')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'localization_sim.launch.py')),
        launch_arguments=sim_args.items())]


def generate_launch_description():
    args = [
        ('test_case', '1', '1 = nominal three-waypoint route, 2 = kidnapping'),
        ('output_dir', '', 'Results, videos and screenshots; default ~/loc_results/test_case_<n>'),
        ('viz', 'true', 'Show the EKF and PF views live'),
        ('viz_window', '16.0', 'Side of the robot-following map view in m; 0 = whole map'),
        ('record_video', 'true', 'Save <filter>.mp4 of each view (ffmpeg or OpenCV) next to the results'),
        ('record_bag', 'false', 'Also record the raw streams for offline replay'),
        ('check_running_sim', 'true', 'Refuse to start while a Gazebo sim or ros_gz bridge is still running'),
        ('gz_gui', 'false', 'Show the Gazebo GUI (if the sim launch supports it)'),
        ('num_particles', '2000', 'PF particle count'),
        ('seed', '0', 'Noise and PF seed'),
        ('kidnap_delay', '10.0', 'Test case 2: sim seconds after reaching Goal 1 before the teleport'),
        ('posterior_times', '15,45', 'Sim seconds to save the PF-particles + EKF-covariance figure '
                                     '(test case 2 also saves 1, 5 and 15 s after the kidnap)'),
        ('max_duration', '400.0', 'End the run after this many sim seconds even if Goal 3 is not reached'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=launch_setup)])
