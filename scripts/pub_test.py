#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import json
import time

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image, PointCloud2, Image,PointField
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped



if __name__ == '__main__':
    rospy.init_node('robot1_msg_transfer', anonymous=True)
   



    pcd_pub = rospy.Publisher('/lidar_center/velodyne_points2', PointCloud2, queue_size=0)
    img_pub = rospy.Publisher('/robot_0/stereo_color/right/image_color2', Image, queue_size=0)
    marker_pub = rospy.Publisher('/detection/lidar_detector/objects_markers2', MarkerArray, queue_size=0)
    tf_pub = rospy.Publisher('/tf2', TFMessage, queue_size=0)

    # rospy.spin()
    rate = rospy.Rate(1000)
    img = cv2.imread('img.jpeg')
    Bridge = CvBridge()
    while not rospy.is_shutdown():
        
        img_pub.publish(Bridge.cv2_to_imgmsg(img, "bgr8"))
        rate.sleep()
