from geometry_msgs.msg import PoseArray, PoseStamped
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    translation_from_matrix,
)
import numpy as np


def pose_to_T(pose):
    T = np.zeros((4, 4))
    T = quaternion_matrix(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    )
    T[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def T_to_pose(T):
    pose = PoseStamped()
    pose.pose.position.x = translation_from_matrix(T)[0]
    pose.pose.position.y = translation_from_matrix(T)[1]
    pose.pose.position.z = translation_from_matrix(T)[2]
    pose.pose.orientation.x = quaternion_from_matrix(T)[0]
    pose.pose.orientation.y = quaternion_from_matrix(T)[1]
    pose.pose.orientation.z = quaternion_from_matrix(T)[2]
    pose.pose.orientation.w = quaternion_from_matrix(T)[3]
    return pose


def generate_T_RM(value: dict) -> np.ndarray:
    T_RO = np.eye(4) 
    tmp = value.get("T_RM", {})
    rotation = tmp.get("rotation", np.eye(3))
    T_RO[0:3, 0:3] = np.array(rotation)
    translation = tmp.get("translation", [0, 0, 0])
    T_RO[0:3, 3] = np.array(translation)
    return T_RO