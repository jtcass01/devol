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
            # Velocity command
            # -----------------
           '/a200_0000/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

            # -----------------
            # Joint States
            # -----------------
           '/a200_0000/dynamic_joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
           '/a200_0000/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',

            # -----------------
            # 2D LiDAR
            # -----------------
            '/a200_0000/sensors/lidar2d_0/scan'
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # -----------------
            # 3D LiDAR
            # -----------------
            '/a200_0000/sensors/lidar3d_0/scan'
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            '/a200_0000/sensors/lidar3d_0/points'
            '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # -----------------
            # Camera (RGB)
            # -----------------
            '/a200_0000/sensors/camera_0/image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',

            '/a200_0000/sensors/camera_0/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # -----------------
            # Camera (Depth)
            # -----------------
            '/a200_0000/sensors/camera_0/depth_image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',

            # -----------------
            # Camera Point Cloud
            # -----------------
            '/a200_0000/sensors/camera_0/points'
            '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
           ],
        remappings=[
           ('/a200_0000/dynamic_joint_states', '/a200_0000/joint_states'),
       ])

   return LaunchDescription([bridge])
