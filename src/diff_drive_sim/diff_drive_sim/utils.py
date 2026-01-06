from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from numpy import ndarray, array, arctan2, pi, cos, sin, zeros

def quaternion_rotation_matrix(Q: Quaternion) -> ndarray:
    """
    Creates a 3x3 rotation matrix in 3D space from a quaternion.

    Input
    :param Q: A 4 element array containing the quaternion (q0,q1,q2,q3) 

    Output
    :return: A 3x3 element matix containing the rotation matrix

    """
    q0, q1, q2, q3 = Q.w, Q.x, Q.y, Q.z

    r_11: float = 1 - 2*(q2**2 + q3**2)
    r_12: float = 2*(q1*q2-q0*q3)
    r_13: float = 2*(q1*q3+q0*q2)
    r_21: float = 2*(q1*q2+q0*q3)
    r_22: float = 1 - 2*(q1**2 + q3**2)
    r_23: float = 2*(q2*q3-q0*q1)
    r_31: float = 2*(q1*q3-q0*q2)
    r_32: float = 2*(q2*q3+q0*q1)
    r_33: float = 1 - 2*(q1**2 + q2**2)

    return array([
        [r_11, r_12, r_13],
        [r_21, r_22, r_23],
        [r_31, r_32, r_33]
    ])


def quaternion_to_euler(Q: Quaternion) -> ndarray:
    """
    Takes a quaternion and returns the roll, pitch yaw array.

    Input
    :param Q0: A 4 element array containing the quaternion (q01,q11,q21,q31) 

    Output
    :return: A 3 element array containing the roll,pitch, and yaw (alpha,beta,gamma) 

    """
    R: ndarray = quaternion_rotation_matrix(Q)

    alpha: float = arctan2(R[2,1], R[2,2])
    beta: float = arctan2(-R[2,0], (R[2,1]**2+R[2,2]**2)**0.5)
    gamma: float = arctan2(R[1,0], R[0,0])

    return array([
        alpha, beta, gamma
    ])