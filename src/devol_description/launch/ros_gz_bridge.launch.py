from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            # Camera (RGB)
            # -----------------
            '/devol/sensors/camera/image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',

            '/devol/sensors/camera/color_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # -----------------
            # Camera (Depth)
            # -----------------
            '/devol/sensors/camera/depth_image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',

            '/devol/sensors/camera/depth_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ]
    )

    return LaunchDescription([bridge])