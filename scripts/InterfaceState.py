import rospy
from funcs import *


class InterfaceState:

    def __init__(self, name, topic, index):

        self.name = name
        
        self.active = False 

        self.W_T_obj = None       # Motion interface pose (current)
        self.W_T_obj_prev = None  # Motion interface pose (initial / previous)
        self.W_T_ref = None       # Motion interface reference pose (initial)
        self.W_T_obj_init = None  

        self.index_interface = index - 1

        # how many msgs to be discarded in the beginning ?
        # counter_msg_ee_pose counts to 5    
        self.counter_msg_interface_pose = 0
        self.valid_interface_pose = False

        self.sb_interface_pose = rospy.Subscriber(
            topic, PoseArray, self.cbk_interface_pose, queue_size=1
        )

    def set_W_T_ref(self, T):
        self.W_T_ref = T

    def get_W_T_ref(self):
        return self.W_T_ref
    
    def set_W_T_obj_prev(self, T):
        self.W_T_obj_prev = T

    def get_W_T_obj_prev(self):
        return self.W_T_obj_prev

    def set_W_T_obj_init(self, T):
        self.W_T_obj_init = T

    def get_W_T_obj_init(self):
        return self.W_T_obj_init

    def set_W_T_obj(self, T):
        self.W_T_obj = T

    def get_W_T_obj(self):
        return self.W_T_obj

    def is_valid(self):
        return self.active
    
    def cbk_interface_pose(self, msg: PoseArray):

        T_current = pose_to_T(msg.poses[self.index_interface])

        if not self.valid_interface_pose:
            self.counter_msg_interface_pose += 1
            if self.counter_msg_interface_pose >= 5:
                self.W_T_obj_prev = T_current
                self.W_T_obj_init = T_current
                self.W_T_ref = pose_to_T(msg.poses[4])
                self.valid_interface_pose = True

        else:
            self.active = True
            self.W_T_obj = T_current

