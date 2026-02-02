import rospy
from funcs import *


class RobotState:

    def __init__(
        self,
        name,
        frame_id,
        topic_ee_pose,
        topic_desired_motions,
        T_RO
    ):

        self.name = name
        self.frame_id = frame_id

        self.T_R = None       # EE pose (current)
        self.T_R_prev = None  # EE pose (initial / previous)

        self.T_des = None       # desired EE pose (current)
        self.T_des_prev = None  # desired EE pose (current)

        # how many msgs to be discarded in the beginning ?
        # counter_msg_ee_pose counts to 5
        self.counter_msg_ee_pose = 0
        self.valid_ee_pose = False

        self.set_T_RO(T_RO)

        self.sb_ee_pose = rospy.Subscriber(
            topic_ee_pose, PoseArray, self.cbk_ee_pose, queue_size=1
        )
        
        self.pb_desired_motions = rospy.Publisher(
            topic_desired_motions, PoseStamped, queue_size=1
        )

    def get_T_RO(self):
        return self.T_RO

    def set_T_R(self, T):
        self.T_R = T

    def get_T_R(self):
        return self.T_R

    def set_T_R_prev(self, T):
        self.T_R_prev = T

    def get_T_R_prev(self):
        return self.T_R_prev

    def set_T_des(self, T):
        self.T_des = T

    def get_T_des(self):
        return self.T_des

    def set_T_des_prev(self, T):
        self.T_des_prev = T

    def get_T_des_prev(self):
        return self.T_des_prev

    def is_valid(self):
        return self.valid_ee_pose
    
    def cbk_ee_pose(self, msg: PoseStamped):

        T_current = pose_to_T(msg.pose)

        if not self.valid_ee_pose:
            self.counter_msg_ee_pose += 1
            if self.counter_msg_ee_pose >= 5:
                self.T_R_prev = T_current
                self.valid_ee_pose = True

        else:
            self.T_R = T_current

    def publish(self, T):
        pose = T_to_pose(T)
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        self.pb_desired_motions.publish(pose)
