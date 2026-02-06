#!/usr/bin/env python3

from funcs import *
from RobotState import RobotState
from InterfaceState import InterfaceState

import rospy
import rospkg

import yaml
import pickle
import os 


LOG = True 
teleop_staterted = False


def init_teleop(iface, rob):

    # interface pose w.r.t. to the optitrack world frame
    W_T_obj_0 = iface.get_W_T_obj()
    W_T_ref = iface.get_W_T_ref()
    ref_T_obj_0 = np.linalg.inv(W_T_ref) @ W_T_obj_0

    # end-effector pose w.r.t. robot's base frame
    B_T_EE_0 = rob.get_T_EE()
    # print("Initial EE pose in robot base frame:\n", T_to_numpy_pose(B_T_EE_0))

    # Expressing the base of robot in the defined ref frame (for calibration)
    ref_T_B = rob.get_ref_T_B()
    # print("ref_T_B:\n", T_to_numpy_pose(ref_T_B))
    B_T_ref = np.linalg.inv(ref_T_B)

    B_T_obj_0 = B_T_ref @ ref_T_obj_0
    # print("Initial object pose in robot base frame:\n", T_to_numpy_pose(B_T_obj_0))

    # Fixed offset
    # obj_T_EE = np.linalg.inv(ref_T_obj_0) @ np.linalg.inv(B_T_ref) @ B_T_EE_0
    obj_T_EE_0 = np.linalg.inv(B_T_obj_0) @ B_T_EE_0
    # print("Initial object to EE pose (offset):\n", T_to_numpy_pose(obj_T_EE_0))

    rob.set_offset(obj_T_EE_0)


def update(iface, rob):
    global teleop_staterted

    if not teleop_staterted:
        teleop_staterted = True 
        init_teleop(iface, rob)
        return np.eye(4)
    
    else: 
        W_T_obj = iface.get_W_T_obj()
        W_T_ref = iface.get_W_T_ref()
        ref_T_obj = np.linalg.inv(W_T_ref) @ W_T_obj

        ref_T_B = rob.get_ref_T_B()
        B_T_ref = np.linalg.inv(ref_T_B)

        B_T_obj = B_T_ref @ ref_T_obj
        # print("Current object pose in robot base frame:\n", T_to_numpy_pose(B_T_obj))

        T_offset = rob.get_offset()
        # print("Offset pose (obj_T_EE):\n", T_to_numpy_pose(T_offset))
        # exit()

        # B_T_EE_des = B_T_ref @ ref_T_obj @ T_offset
        # print("Desired B_T_EE:\n", T_to_numpy_pose(B_T_EE_des))
        B_T_EE_des = B_T_obj @ T_offset
        # print("Desired EE pose in robot base frame:\n", T_to_numpy_pose(B_T_EE_des))
        
        rob.publish(B_T_EE_des)
        print("Desired EE pose in robot base frame:\n", T_to_numpy_pose(B_T_EE_des))
        return B_T_EE_des 
    

if __name__ == "__main__":

    rospy.init_node("motion_mapping_node")

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('robot_control_with_mocap')

    config_path = os.path.join(pkg_path, 'config', 'robots.yaml')
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # for logging with pickle
    lst_interface_log = []  
    lst_robot_log = []

    if LOG:
        log_dir = os.path.join(pkg_path, 'log')
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

    ros_node_rate = config.get("ros_node", {}).get("loop_rate", None)

    robots, interfaces = {}, {}
    # loading robots and interfaces from the YAML file
    for key, value in config.get("robots", {}).items():

        # just keep the active, defined robots in the config file
        if not value.get("name"): continue

        topics = value.get("topics", {}) 
      
        robots[key] = RobotState(
            name=value["name"],
            frame_id=value.get("frame_id", ""),
            topic_ee_pose=topics.get("ee_pose", ""),
            topic_desired_motions=topics.get("desired_motions", ""),
            ref_T_B=generate_ref_T_B(value)
        )

        interfaces[key] = InterfaceState(
            name=value["name"],
            topic=topics.get("motion_interface", ""), 
            index=value.get("motion_interface_index", -1)
        )  

    # starting the ROS node
    rate = rospy.Rate(ros_node_rate)

    teleop_staterted = False
    try: 
        while not rospy.is_shutdown():
            
            # for logging
            interface_log = [] 
            robot_log = []

            for idx, interface_key in enumerate(interfaces):

                robot = robots[interface_key]
                interface = interfaces[interface_key]

                if robot.is_valid() and interface.is_valid():

                    # motion mapping 
                    T_desired = update(interface, robot)
                   
                    if (LOG):

                        # add interface current pose
                        pose = T_to_numpy_pose(interface.get_W_T_obj()) 
                        entry = {"name": interface_key, "pose": pose}
                        interface_log.append(entry)

                        # add robot current pose
                        pose = T_to_numpy_pose(T_desired) 
                        
                        # print(type(pose), pose.shape)
                        # print("pose:", pose )
                        entry = {"name": interface_key, "pose": pose}
                        robot_log.append(entry)


            # update the lists of pose information for interfaces and robots
            if (LOG):
                lst_interface_log.append(interface_log)
                lst_robot_log.append(robot_log)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:

        # save the log variables as pickle files
        if LOG:

            with open(os.path.join(log_dir, "lst_robot_log.pkl"), "wb") as f:
                pickle.dump(lst_robot_log, f)

            with open(os.path.join(log_dir, "lst_interface_log.pkl"), "wb") as f:
                pickle.dump(lst_interface_log, f)

            print("Pickle logs saved in 'log/' folder.")        