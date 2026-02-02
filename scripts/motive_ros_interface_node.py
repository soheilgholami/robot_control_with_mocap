#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header

import socket
import threading
import numpy as np

UDP_IP = "127.0.0.1"
UDP_PORT_MOCAP_TRIO_1 = 5005
UDP_PORT_MOCAP_TRIO_2 = 5007
UDP_PORT_MOCAP_FIXED = 5009
ROS_NODE_FREQ = 120.0
# if True: using two trio cameras, else uses the non portable one
MOCAP_PORTABLE = False

# store pose data from 2 mocap system, each with 2 objects
mocap_data = np.zeros((4, 7))

lock = threading.Lock()


def udp_mocap_trio_1():
    global mocap_data
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind((UDP_IP, UDP_PORT_MOCAP_TRIO_1))
        while True:
            data, _ = server_socket.recvfrom(1024)
            parts = data.decode().split(":")
            idx = int(parts[0]) - 1
            with lock:
                mocap_data[idx, :3] = eval(parts[1])
                mocap_data[idx, 3:] = eval(parts[2])


def udp_mocap_trio_2():
    global mocap_data
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind((UDP_IP, UDP_PORT_MOCAP_TRIO_2))
        while True:
            data, _ = server_socket.recvfrom(1024)
            parts = data.decode().split(":")
            idx = int(parts[0]) - 1
            with lock:
                mocap_data[idx, :3] = eval(parts[1])
                mocap_data[idx, 3:] = eval(parts[2])


def udp_mocap_fixed():
    global mocap_data
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind((UDP_IP, UDP_PORT_MOCAP_FIXED))
        while True:
            data, _ = server_socket.recvfrom(1024)
            parts = data.decode().split(":")
            idx = int(parts[0]) - 1
            with lock:
                mocap_data[idx, :3] = eval(parts[1])
                mocap_data[idx, 3:] = eval(parts[2])


def ros_node():
    global mocap_data
    mocap_data = np.zeros_like(mocap_data)

    if (MOCAP_PORTABLE):
        threading.Thread(target=udp_mocap_trio_1, daemon=True).start()
        threading.Thread(target=udp_mocap_trio_2, daemon=True).start()
    else: 
        threading.Thread(target=udp_mocap_fixed, daemon=True).start()

    rospy.init_node("motive_ros_interface_node")

    pub = rospy.Publisher("/mocap_system/pose_arrays", 
                          PoseArray, 
                          queue_size=1)

    rate = rospy.Rate(ROS_NODE_FREQ)

    rospy.loginfo(f">> motive_ros_interface_node is running...")

    while not rospy.is_shutdown():

        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "mocap_base_frame"

        with lock:
            for i in range(4):
                p = Pose()
                p.position.x = mocap_data[i, 0]
                p.position.y = mocap_data[i, 1]
                p.position.z = mocap_data[i, 2]
                p.orientation.x = mocap_data[i, 3]
                p.orientation.y = mocap_data[i, 4]
                p.orientation.z = mocap_data[i, 5]
                p.orientation.w = mocap_data[i, 6]
                msg.poses.append(p)

        pub.publish(msg)
        rate.sleep()



if __name__ == "__main__":

    try:
        ros_node()

    except rospy.ROSInterruptException:
        pass
