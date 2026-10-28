from typing import Union
from numpy import ndarray, pi, sin, cos, atan2, array

from geometry_msgs.msg import Quaternion, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


def euler_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=sin(yaw / 2.0), w=cos(yaw / 2.0))


def yaw_from_quaternion(q):
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(x: ndarray) -> ndarray:
    """Wrap heading to [-pi, pi] to prevent runaway angle values over time"""
    x[2] = (x[2] + pi) % (2 * pi) - pi
    return x


def compose(a: ndarray, b: ndarray) -> ndarray:
    """2D Pose composition: a ⊕ b"""
    sin_a: float = sin(a[2])
    cos_a: float = cos(a[2])
    return array(
        [
            a[0] + cos_a * b[0] - sin_a * b[1],
            a[1] + sin_a * b[0] + cos_a * b[1],
            (a[2] + b[2] + pi) % (2 * pi) - pi,
        ]
    )


def inverse(a: ndarray) -> ndarray:
    """2D Pose inversion: a^-1"""
    sin_a: float = sin(a[2])
    cos_a: float = cos(a[2])
    return array([-cos_a * a[0] - sin_a * a[1], sin_a * a[0] - cos_a * a[1], -a[2]])


def pose_to_planar(pose: Union[Pose, PoseWithCovarianceStamped]) -> ndarray:
    if isinstance(pose, PoseWithCovarianceStamped):
        pose = pose.pose.pose
    return array([pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)])


def odom_to_planar(msg: Odometry) -> ndarray:
    return pose_to_planar(msg.pose.pose)
