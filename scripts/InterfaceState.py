import rospy
from funcs import *


class InterfaceState:

    def __init__(self, name, topic, index):

        self.name = name

        self.T_M = None       # Motion interface pose (current)
        self.T_M_prev = None  # Motion interface pose (initial / previous)
        
        self.index_interface = index - 1

        # how many msgs to be discarded in the beginning ?
        # counter_msg_ee_pose counts to 5    
        self.counter_msg_interface_pose = 0
        self.valid_interface_pose = False

        self.sb_interface_pose = rospy.Subscriber(
            topic, PoseArray, self.cbk_interface_pose, queue_size=1
        )

    def set_T_M_prev(self, T):
        self.T_M_prev = T

    def get_T_M_prev(self):
        return self.T_M_prev

    def set_T_M(self, T):
        self.T_M = T

    def get_T_M(self):
        return self.T_M

    def is_valid(self):
        return self.valid_interface_pose
    
    def cbk_interface_pose(self, msg: PoseArray):

        T_current = pose_to_T(msg.poses[self.index_interface])

        if not self.valid_interface_pose:

            self.counter_msg_interface_pose += 1
            if self.counter_msg_interface_pose >= 5:
                self.T_M_prev = T_current
                self.valid_interface_pose = True

        else:
            self.T_M = T_current

