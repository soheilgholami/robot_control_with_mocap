#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header

import socket
import threading
import numpy as np

import os
import pickle
import rospkg

UDP_IP = "127.0.0.1"
UDP_PORT_MOCAP_TRIO_1 = 5005
UDP_PORT_MOCAP_TRIO_2 = 5007
UDP_PORT_MOCAP_FIXED = 5009
ROS_NODE_FREQ = 250.0

# if True: using two trio cameras, else uses the non portable one
MOCAP_PORTABLE = False

# store pose data from 2 mocap system, each with 2 objects + ref markerset
mocap_data = np.zeros((5, 7))

mocap_data_raw = []
mocap_data_fil = []


WINDOW_SIZE = 15  # number of frames to average over
# buffer for each marker: shape (5 markers, WINDOW_SIZE, 7)
mocap_buffer = np.zeros((5, WINDOW_SIZE, 7))
buffer_index = 0
buffer_full = False

lock = threading.Lock()

def get_log_folder(package_name="robot_control_with_mocap"):
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(package_name)
    log_path = os.path.join(pkg_path, "log")

    if not os.path.exists(log_path):
        os.makedirs(log_path)
        print(f"Created log folder at: {log_path}")

    return log_path

def udp_mocap_trio_1():
    global mocap_data
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind((UDP_IP, UDP_PORT_MOCAP_TRIO_1))
        while True:
            data, _ = server_socket.recvfrom(1024)
            parts = data.decode().split(":")
            idx = int(parts[0]) - 1
            if idx < 0 or idx > 2: continue
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
            if idx < 0 or idx > 2: continue
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
            if idx < 0 or idx > 4: continue
            with lock:
                mocap_data[idx, :3] = eval(parts[1])
                mocap_data[idx, 3:] = eval(parts[2])


def ros_node():
    global mocap_data, mocap_data_raw, mocap_data_fil
    global mocap_buffer, buffer_index, buffer_full

    mocap_data = np.zeros_like(mocap_data)
    mocap_data_raw = []
    mocap_data_fil = []
    mocap_buffer = np.zeros_like(mocap_buffer)
    buffer_index = 0
    buffer_full = False

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
            # update the circular buffer
            mocap_buffer[:, buffer_index, :] = mocap_data
            buffer_index = (buffer_index + 1) % WINDOW_SIZE
            if buffer_index == 0:
                buffer_full = True

            # compute the moving average
            if buffer_full:
                mocap_avg = np.mean(mocap_buffer, axis=1)
            else:
                mocap_avg = np.mean(mocap_buffer[:, :buffer_index, :], axis=1)

            # fill the PoseArray
            for i in range(5):
                p = Pose()
                p.position.x = mocap_avg[i, 0]
                p.position.y = mocap_avg[i, 1]
                p.position.z = mocap_avg[i, 2]
                p.orientation.x = mocap_avg[i, 3]
                p.orientation.y = mocap_avg[i, 4]
                p.orientation.z = mocap_avg[i, 5]
                p.orientation.w = mocap_avg[i, 6]
                msg.poses.append(p)

            mocap_data_raw.append(mocap_data.copy())  # raw snapshot
            mocap_data_fil.append(mocap_avg.copy())   # filtered snapshot

        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":

    try:
        ros_node()

    except rospy.ROSInterruptException:
        pass

    finally:

        log_folder = get_log_folder("robot_control_with_mocap") 

        # save raw and filtered mocap data
        raw_file = os.path.join(log_folder, "mocap_raw.pkl")
        filtered_file = os.path.join(log_folder, "mocap_filtered.pkl")

        with open(raw_file, "wb") as f:
            pickle.dump(mocap_data_raw, f)

        with open(filtered_file, "wb") as f:
            pickle.dump(mocap_data_fil, f)

        print(f"Saved raw mocap data to {raw_file}")
        print(f"Saved filtered mocap data to {filtered_file}")
