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


robot_num = 1
client_e2r_dict = {}
client_r2e_dict = {}

def start_recv(id):

    ClientRecv(config = './config/config-robot-recv-3.yaml', robot_id = id)


def start_send(id):
 
    ClientSend(config = './config/config-robot-send-3.yaml', robot_id = id)


if __name__ == '__main__':
    robot_id =3
    rospy.init_node('robot_'+str(robot_id)+'msg_transfer', anonymous=True)

    
    start_recv_thread = threading.Thread(
        target = start_recv(robot_id), args=()
    )
    start_send_thread = threading.Thread(
        target = start_send(robot_id), args=()
    )
    start_recv_thread.start()
    start_send_thread.start()
    
    while True & (not rospy.is_shutdown()):
        rospy.spin()
        time.sleep(0.01)

