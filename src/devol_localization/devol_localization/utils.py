
import numpy as np
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=np.sin(yaw/2.0), w=np.cos(yaw/2.0))

def yaw_from_quaternion(q):
    return np.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def compose(a, b):
    """2D Pose composition: a ⊕ b"""
    sin_a: float = np.sin(a[2])
    cos_a: float = np.cos(a[2])
    return np.array([a[0] + cos_a * b[0] - sin_a * b[1], 
                     a[1] + sin_a * b[0] + cos_a * b[1], 
                     (a[2] + b[2] + np.pi) % (2 * np.pi) - np.pi])

def inverse(a):
    """2D Pose inversion: a^-1"""
    sin_a: float = np.sin(a[2])
    cos_a: float = np.cos(a[2])
    return np.array([-cos_a * a[0] - sin_a * a[1], 
                     sin_a * a[0] - cos_a * a[1], 
                     -a[2]])
