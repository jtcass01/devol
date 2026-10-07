"""ROS <-> Gazebo bridge for the robot's drive, joint states and sensors under <namespace>.

Topic names follow the plugins and sensors in the a200 and devol URDFs (a200.gazebo.xacro,
devol.gazebo.xacro). Topics whose sensor is not in the model (for example the cameras with
use_cameras:=false) are bridged but stay silent.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    ns = LaunchConfiguration('namespace').perform(context)
    topics = [
        'cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        'dynamic_joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        'odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        'sensors/lidar2d_0/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        'sensors/lidar3d_0/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        'sensors/lidar3d_0/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
    ]
    # camera_0: the base's rear RealSense D435; camera_1: the wrist D405.
    for cam in ('camera_0', 'camera_1'):
        topics += [
            f'sensors/{cam}/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'sensors/{cam}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'sensors/{cam}/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'sensors/{cam}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ]
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[f'{ns}/{t}' for t in topics],
            remappings=[(f'{ns}/dynamic_joint_states', f'{ns}/joint_states')],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace', default_value='/devol_drive', description='Namespace for topics'
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
