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

from cv_bridge.boost.cv_bridge_boost import getCvType


if __name__ == '__main__':
    rospy.init_node('robot_0_pub', anonymous=True)
    robot_id = 2

    debug = False
    #pcd_pub = rospy.Publisher('/robot_'+str(robot_id)+'/lidar_center/velodyne_points2', PointCloud2, queue_size=0)
    img_pub = rospy.Publisher('/robot_'+str(robot_id)+'/stereo_color/right/image_color', Image, queue_size=0)
    #marker_pub = rospy.Publisher('/robot_'+str(robot_id)+'/detection/lidar_detector/objects_markers2', MarkerArray, queue_size=0)
    #tf_pub = rospy.Publisher('/robot_'+str(robot_id)+'/tf2', TFMessage, queue_size=0)

    # rospy.spin()
    rate = rospy.Rate(10)
    img = cv2.imread('img.jpg')
    print(img.shape)
    img = cv2.resize(img,(640, 480))
   
    Bridge = CvBridge()
    print("publish img")
    while not rospy.is_shutdown():
        if debug:
            cv2.imshow('pub_'+str(robot_id),img)
            cv2.waitKey(1)
        img_pub.publish(Bridge.cv2_to_imgmsg(img, "bgr8"))
        rate.sleep()
