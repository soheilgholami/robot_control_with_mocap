import rospy
from funcs import *

from franka_msgs.msg import FrankaState

class RobotState:

    def __init__(
        self,
        name,
        frame_id,
        topic_ee_pose,
        topic_desired_motions,
        ref_T_B
    ):

        self.name = name
        self.frame_id = frame_id

        self.T_EE = None       # EE pose (current)
        self.T_EE_prev = None  # EE pose (initial / previous)
        self.T_EE_init = None

        self.T_offset = None 

        self.T_EE_des = None       # desired EE pose (current)
        self.T_EE_des_prev = None  # desired EE pose (current)

        # how many msgs to be discarded in the beginning ?
        # counter_msg_ee_pose counts to 5
        self.counter_msg_ee_pose = 0
        self.valid_ee_pose = False

        self.active = False

        self.set_ref_T_B(ref_T_B)

        self.sb_ee_pose = rospy.Subscriber(
            topic_ee_pose, FrankaState, self.cbk_ee_pose, queue_size=1
        )

        self.pb_desired_motions = rospy.Publisher(
            topic_desired_motions, PoseStamped, queue_size=1
        )

    def set_ref_T_B(self, T):
        self.ref_T_B = T

    def get_ref_T_B(self):
        return self.ref_T_B

    def set_T_EE(self, T):
        self.T_EE = T

    def get_T_EE(self):
        return self.T_EE

    def set_T_EE_prev(self, T):
        self.T_EE_prev = T

    def get_T_EE_prev(self):
        return self.T_EE_prev

    def set_T_EE_init(self, T):
        self.T_EE_init = T

    def get_T_EE_init(self):
        return self.T_EE_init

    def set_T_EE_des(self, T):
        self.T_EE_des = T

    def get_T_EE_des(self):
        return self.T_EE_des

    def set_T_EE_des_prev(self, T):
        self.T_EE_des_prev = T

    def get_T_EE_des_prev(self):
        return self.T_EE_des_prev


    def set_offset(self, T):
        self.T_offset = T

    def get_offset(self):
        return self.T_offset

    def is_valid(self):
        return self.active

    def cbk_ee_pose(self, msg: FrankaState):

        T_current = np.array(msg.O_T_EE).reshape((4, 4), order='F')

        if not self.valid_ee_pose:
            self.counter_msg_ee_pose += 1
            if self.counter_msg_ee_pose >= 5:
                self.T_EE_prev = T_current
                self.T_EE_init = T_current
                self.valid_ee_pose = True

        else:
            self.active = True
            self.T_EE = T_current


    def publish(self, T):
        pose = T_to_pose(T)
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        self.pb_desired_motions.publish(pose)
