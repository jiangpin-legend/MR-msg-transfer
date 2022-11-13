#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import json
import time

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image, PointCloud2, Image,PointField
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

from informer import Informer


class Client(Informer):
    def send_msg(self, message):
        self.send(message, 'msg')
    
    def send_odm(self, message):
        self.send(message, 'odm')

    def send_pcd(self, message):
        self.send(message, 'pcd')

    def send_img(self, message):
        self.send(message, 'img')

    def msg_recv(self):
        self.recv('msg', parse_msg)

    def pcd_recv(self):
        self.recv('pcd', parse_sem_pcd)

    def img_recv(self):
        self.recv('img', parse_img)

    def odm_recv(self):
        self.recv('odm', parse_odm)


def callback_message(ros_marker_array):
    global ifm
    cnt = 0
    data_dict = {}
    for ros_marker in ros_marker_array.markers:
        mark_json = {
            'id':ros_marker.id,
            'type': ros_marker.type,
            'x': ros_marker.pose.position.x,
            'y': ros_marker.pose.position.y,
            'z': ros_marker.pose.position.z,
            'ox': ros_marker.pose.orientation.x,
            'oy': ros_marker.pose.orientation.y,
            'oz': ros_marker.pose.orientation.z,
            'ow': ros_marker.pose.orientation.w,
            'sx': ros_marker.scale.x,
            'sy': ros_marker.scale.y,
            'sz': ros_marker.scale.z,
            'r': ros_marker.color.r,
            'g': ros_marker.color.g,
            'b': ros_marker.color.b,
            'a': ros_marker.color.a,
        }
        data_dict[cnt] = mark_json
        cnt += 1
    sent_data = json.dumps(data_dict).encode()
    ifm.send_msg(sent_data)

def callback_pcd(ros_pcd : PointCloud2):
    global ifm
    # 靠5G发送
    ifm.send_pcd(ros_pcd.data)

def callback_odometry(ros_tf):
    global ifm
    cnt = 0
    data_dict = {}
    for tfmessage in ros_tf.transforms:
        tf_json = {
            'x' : tfmessage.transform.translation.x,
            'y' : tfmessage.transform.translation.y,
            'z' : tfmessage.transform.translation.z,
            'rx' : tfmessage.transform.rotation.x,
            'ry' : tfmessage.transform.rotation.y,
            'rz' : tfmessage.transform.rotation.z,
            'rw' : tfmessage.transform.rotation.w,
        }
        data_dict[cnt] = tf_json
        cnt += 1
    sent_data = json.dumps(data_dict).encode()
    # print('send odm')
    ifm.send_odm(sent_data)

def callback_img(ros_img : Image):
    global ifm
    # ifm.send_img(ros_img.data)
    img = np.ndarray(shape=(768, 2048, 3), dtype=np.dtype("uint8"), buffer=ros_img.data)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (2048//2, 768//2), interpolation = cv2.INTER_AREA)
    _, jpeg = cv2.imencode('.jpg', img)
    data = jpeg.tobytes()
    # print('send img', len(data))
    ifm.send_img(data)

def parse_odm(message, robot_id):
    # print('recv odm', len(message))
    data = str(message, encoding = "utf-8")
    json_data = json.loads(data)

    ros_tf = TFMessage()

    for key in json_data.keys():
        json_tf = json_data[key]
        t = TransformStamped()
        t.header.frame_id = "reference_frame"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "odom"
        t.transform.translation.x = json_tf['x']
        t.transform.translation.y = json_tf['y']
        t.transform.translation.z = json_tf['z']
        t.transform.rotation.x = json_tf['rx']
        t.transform.rotation.y = json_tf['ry']
        t.transform.rotation.z = json_tf['rz']
        t.transform.rotation.w = json_tf['rw']

        ros_tf.transforms.append(t)
    
    tf_pub.publish(ros_tf)

def parse_img(message, robot_id):
    # print('img', len(message))
    nparr = np.frombuffer(message, np.uint8)
    img_data = cv2.imdecode(nparr,  cv2.IMREAD_COLOR)
    # img_data = cv2.cvtColor(img_data, cv2.COLOR_BGR2RGB)
    img_data = np.reshape(img_data, ((2048//2)*(768//2)*3, 1))
    img = Image()
    img.data = img_data.tobytes()
    img.height = 768//2
    img.width = 2048//2
    img.encoding = "bgr8"
    img.is_bigendian = 0
    img.step = 6144

    img_pub.publish(img)


def parse_msg(message, robot_id):
    data = str(message, encoding = "utf-8")
    json_data = json.loads(data)
    # print(json_data)
    marker_list = MarkerArray()
    for key in json_data.keys():
        json_marker = json_data[key]
        ros_marker = Marker()
        ros_marker.header = Header()
        ros_marker.header.frame_id = 'lidar_center'
        ros_marker.id = json_marker['id']
        ros_marker.type = json_marker['type']
        ros_marker.pose.position.x = json_marker['x']
        ros_marker.pose.position.y = json_marker['y']
        ros_marker.pose.position.z = json_marker['z']
        ros_marker.pose.orientation.x = json_marker['ox']
        ros_marker.pose.orientation.y = json_marker['oy']
        ros_marker.pose.orientation.z = json_marker['oz']
        ros_marker.pose.orientation.w = json_marker['ow']
        ros_marker.scale.x = json_marker['sx']
        ros_marker.scale.y = json_marker['sy']
        ros_marker.scale.z = json_marker['sz']
        ros_marker.color.r = json_marker['r']
        ros_marker.color.g = json_marker['g']
        ros_marker.color.b = json_marker['b']
        ros_marker.color.a = json_marker['a']
        ros_marker.frame_locked = False
        ros_marker.mesh_use_embedded_materials = False
        ros_marker.text = str(ros_marker.id)

        marker_list.markers.append(ros_marker)
    
    marker_pub.publish(marker_list)

def parse_sem_pcd(message, robot_id):
    pcd = PointCloud2()
    pcd.header = Header()
    # pcd.header.stamp = rospy.Time.now()
    pcd.header.frame_id = 'lidar_center'
    pcd.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
    ]
    pcd.data = message
    pcd.point_step = 16
    pcd.width = len(pcd.data)//pcd.point_step
    pcd.height = 1
    pcd.row_step = len(pcd.data)
    pcd.is_bigendian = False
    pcd.is_dense = True

    pcd_pub.publish(pcd)


if __name__ == '__main__':
    rospy.init_node('robot1_msg_transfer', anonymous=True)
    ifm = Client(config = 'config-robot-1.yaml')
    ################################################################
    # 如果有报错，说这个TCP还没建立好，conn里key没有，就在这里sleep 0.5秒
    #################################################################
    # 收的topic名字和处理的回调函数

    rospy.Subscriber('/tf', TFMessage, callback_odometry)
    rospy.Subscriber('/point_cloud2', PointCloud2, callback_pcd)
    rospy.Subscriber('/robot_1/stereo_color/right/image_color', Image, callback_img)
    rospy.Subscriber('/detection/lidar_detector/objects_markers', MarkerArray, callback_message)

    pcd_pub = rospy.Publisher('/lidar_center/velodyne_points2', PointCloud2, queue_size=0)
    img_pub = rospy.Publisher('/robot_1/stereo_color/right/image_color2', Image, queue_size=0)
    marker_pub = rospy.Publisher('/detection/lidar_detector/objects_markers2', MarkerArray, queue_size=0)
    tf_pub = rospy.Publisher('/tf2', TFMessage, queue_size=0)


    # rospy.spin()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
