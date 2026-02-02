#!/usr/bin/env python3

from funcs import *
from RobotState import RobotState
from InterfaceState import InterfaceState

import rospy
import yaml
import pickle


def update(iface, rob):
        
        # transformation matrix : robot and motion interface base frames
        T_RM = rob.get_T_RM()
        
        # get current pose of the motion interface (T matrix)
        T_M = iface.get_T_M()
        
        # relative displacement compared to the past pose
        T_M_rel = np.linalg.inv(iface.get_T_M_prev()) @ T_M
        # transform it
        T_R_rel = T_RM @ T_M_rel @ np.linalg.inv(T_RM)
        
        # calculating the current robot pose
        T_R_prev = rob.get_T_R_prev()
        T_des = T_R_prev @ T_R_rel 

        return T_des


if __name__ == "__main__":

    with open("config/robots.yaml", "r") as f:
        config = yaml.safe_load(f)

    ros_node_rate = config.get("ros_node", {}).get("loop_rate", None)

    robots, interfaces = {}, {}
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
            T_RM=generate_T_RM(value)
        )

        interfaces[key] = InterfaceState(
            name=value["name"],
            topic=topics.get("motion_interface", ""), 
            index=value.get("motion_interface_idndex", -1)
        )  

    # starting the ROS node
    rate = rospy.Rate(ros_node_rate)

    while not rospy.is_shutdown():

        for idx, interface_key in enumerate(interfaces):

            robot = robots[interface_key]
            interface = interfaces[interface_key]

            if robot.is_valid() and interface.isvalid():

                # motion mapping 
                T_desired = update(interface, robot)

                # send command to robot
                robot.publish(T_desired)

                # save the previous variables
                interface.set_T_M_prev(interface.get_T_M())
                robot.set_T_R_prev(robot.get_T_R())
                robot.set_T_des_prev(T_desired)

        rate.sleep()
