from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='/a200_0000',
        description='Namespace for topics'
    )

    def launch_setup(context):
        namespace = context.perform_substitution(LaunchConfiguration('namespace'))

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
                parameters=[
                    {
                        'use_sim_time': True,
                        'qos_overrides./tf.publisher.durability': 'transient_local',
                        'qos_overrides./tf_static.publisher.durability': 'transient_local',
                    }
                ],
            arguments=[
                # -----------------
                # Velocity command
                # -----------------
                f'{namespace}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

                # -----------------
                # Joint States
                # -----------------
                f'{namespace}/dynamic_joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                f'{namespace}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',

                # -----------------
                # 2D LiDAR
                # -----------------
                f'{namespace}/sensors/lidar2d_0/scan'
                '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

                # -----------------
                # 3D LiDAR
                # -----------------
                f'{namespace}/sensors/lidar3d_0/scan'
                '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

                # -----------------
                # Camera (RGB)
                # -----------------
                f'{namespace}/sensors/camera_0/image'
                '@sensor_msgs/msg/Image[gz.msgs.Image',

                f'{namespace}/sensors/camera_0/camera_info'
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

                # -----------------
                # Camera (Depth)
                # -----------------
                f'{namespace}/sensors/camera_0/depth_image'
                '@sensor_msgs/msg/Image[gz.msgs.Image',

                f'{namespace}/sensors/camera_0/points'
                '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            remappings=[
                (f'{namespace}/dynamic_joint_states', f'{namespace}/joint_states'),
            ])


        return [
            bridge
        ]

    return LaunchDescription([declare_namespace,
        OpaqueFunction(function=launch_setup)])
