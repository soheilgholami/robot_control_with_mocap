#!/usr/bin/env python3

from funcs import *
from RobotState import RobotState
from InterfaceState import InterfaceState
import rospy
import yaml


def get_T_cmd(T_RO, T_R_prev, T_O_prev, T_O):
    T_O_rel = np.linalg.inv(T_O_prev) @ T_O
    T_R_rel = T_RO @ T_O_rel @ np.linalg.inv(T_RO)
    return T_R_prev @ T_R_rel


def update(iface, rob):
        
        T_O = iface.get_T_O()
        T_RO = rob.get_T_RO()

        T_O_prev = iface.get_T_O_prev()
        T_R_prev = rob.get_T_R_prev()

        T = get_T_cmd(T_RO, T_R_prev, T_O_prev, T_O)

        rob.set_T_R_prev(T_R_prev)
        rob.set_T_cmd_prev(T)
        iface.set_T_O_prev(T_O)

        return T


if __name__ == "__main__":

    with open("config/robots.yaml", "r") as f:
        config = yaml.safe_load(f)

    robots, interfaces = {}, {}

    FREQ_LOOP = config.get("ros_node", {}).get("loop_rate", None)

    # loading robots and interfaces from the YAML file
    for key, value in config.get("robots", {}).items():

        # just keep the active robots
        if not value.get("name"): continue

        topics = value.get("topics", {}) 
      
        robots[key] = RobotState(
            name=value["name"],
            frame_id=value.get("frame_id", ""),
            topic_ee_pose=topics.get("ee_pose", ""),
            topic_desired_motions=topics.get("desired_motions", ""),
            T_RO=generate_T_RO(value)
        )

        interfaces[key] = InterfaceState(
            name=value["name"],
            topic=topics.get("motion_interface", "")
        )  

    rate = rospy.Rate(FREQ_LOOP)

    while not rospy.is_shutdown():

        for idx, interface_key in enumerate(interfaces):

            robot = robots[interface_key]
            interface = interfaces[interface_key]

            if robot.is_valid() and interface.isvalid():

                T = update(interface, robot)
                # TODO update the interafce and robot values 

                robot.publish(T)

        rate.sleep()
