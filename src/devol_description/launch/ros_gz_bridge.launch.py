from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = DeclareLaunchArgument(
        'namespace',
        default_value=f'devol/',
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
                # Camera (RGB)
                # -----------------
                f'{namespace}/sensors/camera/image'
                '@sensor_msgs/msg/Image[gz.msgs.Image',

                f'{namespace}/sensors/camera/color_info'
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

                # -----------------
                # Camera (Depth)
                # -----------------
                f'{namespace}/sensors/camera/depth_image'
                '@sensor_msgs/msg/Image[gz.msgs.Image',

                f'{namespace}/sensors/camera/depth_info'
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ]
        )

        return [
            bridge
        ]


    return LaunchDescription([namespace,
        OpaqueFunction(function=launch_setup)])
