#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# import cv2
import numpy as np
import json
import time

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image, PointCloud2, Image,PointField
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from dislam_msgs.msg import SubMap

from informer import Informer
from client import ClientRecv, ClientSend
import threading


robot_num = 3
client_recv_dict = {}
client_send_dict = {}

def start_recv():
    global client_recv_dict
    #master robot recv message for own rostopic,so start recv from second robot 
    for i in range(1, robot_num+1):
        client_recv_dict[i] = ClientRecv(config = './config/config-edge-recv.yaml', robot_id = i,isMaster=True)


def start_send():
    global client_send_dict
    #master robot recv message for own rostopic,so start recv from second robot 
    for i in range(1, robot_num+1):
        client_send_dict[i] = ClientSend(config = './config/config-edge-send-'+str(i)+'.yaml', robot_id = i,isMaster=True)


if __name__ == '__main__':
    rospy.init_node('vehicle_5g_transfer', anonymous=True)


    start_recv_thread = threading.Thread(
        target = start_recv, args=()
    )


    start_send_thread = threading.Thread(
        target = start_send, args=()
    )

    start_recv_thread.start()
    start_send_thread.start()

    while True & (not rospy.is_shutdown()):
        rospy.spin()
        time.sleep(0.01)

