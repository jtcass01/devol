"""Check a robot description by eye, in RViz alone or spawned in Gazebo.

  ros2 launch devol_drive_description view.launch.py                 # full robot, RViz + joint sliders
  ros2 launch devol_drive_description view.launch.py model:=a200      # base only (or model:=devol, arm only)
  ros2 launch devol_drive_description view.launch.py gz:=true         # spawned in Gazebo's empty world
  ros2 launch devol_drive_description view.launch.py gz:=true world:=factory use_cameras:=true

Both modes build the URDF the simulation uses (use_gazebo:=true). RViz only (gz:=false) moves the
joints with joint_state_publisher_gui (jsp_gui:=false for the plain publisher). gz:=true spawns the
model at the 'robot' pose of the world's poses.csv, and RViz shows the joint states and lidar scans
Gazebo publishes, with odom as the fixed frame for the drive models.
"""

import csv
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# model: (package, top-level xacro, namespace its Gazebo plugins use, root link, DiffDrive TF topic)
MODELS = {
    'devol_drive': (
        'devol_drive_description',
        'devol_drive.urdf.xacro',
        'devol_drive',
        'a200_base_link',
        '/model/devol_drive/tf',
    ),
    'a200': (
        'a200_description',
        'a200.urdf.xacro',
        'a200_0000',
        'a200_base_link',
        '/model/a200_0000/tf',
    ),
    'devol': ('devol_description', 'devol.urdf.xacro', 'devol', 'world', ''),
}

ARGS = {
    'model': (
        'devol_drive',
        'devol_drive (base + arm), a200 (base) or devol (arm on a fixed world link)',
    ),
    'gz': ('false', 'Spawn the model in Gazebo; false shows it in RViz only'),
    'rviz': ('true', 'Open RViz'),
    'jsp_gui': ('true', 'gz:=false: move the joints with sliders (joint_state_publisher_gui)'),
    'world': ('empty', 'gz:=true: world folder in devol_gazebo/worlds'),
    'gz_gui': ('true', 'gz:=true: open the Gazebo GUI'),
    'use_cameras': ('false', 'gz:=true, model:=devol_drive: simulate the RGB-D cameras'),
}


def launch_setup(context):
    a = {name: LaunchConfiguration(name).perform(context) for name in ARGS}
    if a['model'] not in MODELS:
        raise RuntimeError(f'model must be one of {", ".join(MODELS)}, got {a["model"]}')
    package, xacro_file, ns, root, odom_tf_topic = MODELS[a['model']]
    gz = a['gz'] == 'true'
    # use_gazebo:=false selects the real-hardware ros2_control blocks (the UR driver), which this
    # workspace does not use; the Gazebo build is the URDF the sim runs, so RViz shows that one too.
    xacro_args = ' use_gazebo:=true'
    if a['model'] == 'devol_drive':
        xacro_args += f' use_cameras:={a["use_cameras"]}'

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[
                {
                    'use_sim_time': gz,
                    'robot_description': ParameterValue(
                        Command(
                            [
                                FindExecutable(name='xacro'),
                                ' ',
                                os.path.join(
                                    get_package_share_directory(package), 'urdf', xacro_file
                                ),
                                xacro_args,
                            ]
                        ),
                        value_type=str,
                    ),
                }
            ],
        )
    ]

    if not gz:
        jsp = 'joint_state_publisher_gui' if a['jsp_gui'] == 'true' else 'joint_state_publisher'
        actions.append(Node(package=jsp, executable=jsp, namespace=ns, output='screen'))
        fixed_frame = root
    else:
        world_dir = os.path.join(get_package_share_directory('devol_gazebo'), 'worlds', a['world'])
        with open(os.path.join(world_dir, 'poses.csv')) as f:
            pose = next(row for row in csv.DictReader(f) if row['name'] == 'robot')
        gz_args = ('-r ' if a['gz_gui'] == 'true' else '-r -s ') + os.path.join(
            world_dir, 'maze_world.sdf'
        )
        bridge_topics = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
        if odom_tf_topic:
            bridge_topics.append(f'{odom_tf_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )
                ),
                launch_arguments={'gz_args': gz_args}.items(),
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=['-topic', f'/{ns}/robot_description', '-name', ns]
                + ['-x', pose['x'], '-y', pose['y'], '-z', pose['z'], '-Y', pose['yaw']],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge_system',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=bridge_topics,
                remappings=[(odom_tf_topic, '/tf')] if odom_tf_topic else [],
            ),
        ]
        if odom_tf_topic:
            # Drive models: odometry, joint states and sensors under /<ns>.
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('devol_drive_description'),
                            'launch',
                            'ros_gz_bridge.launch.py',
                        )
                    ),
                    launch_arguments={'namespace': f'/{ns}'}.items(),
                )
            )
            fixed_frame = 'odom'
        else:
            # The arm alone publishes joint states through gz_ros2_control's broadcaster.
            actions.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=['joint_state_broadcaster', '-c', f'/{ns}/controller_manager'],
                )
            )
            fixed_frame = root

    if a['rviz'] == 'true':
        with open(
            os.path.join(
                get_package_share_directory('devol_drive_description'), 'rviz', 'view.rviz'
            )
        ) as f:
            config = f.read().replace('@NS@', ns)
        with tempfile.NamedTemporaryFile('w', suffix='.rviz', delete=False) as f:
            f.write(config)
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', f.name, '-f', fixed_frame],
                parameters=[{'use_sim_time': gz}],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, (d, h) in ARGS.items()]
        + [OpaqueFunction(function=launch_setup)]
    )
