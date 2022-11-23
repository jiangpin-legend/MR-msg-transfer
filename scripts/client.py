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
import threading




class ClientSend(Informer):
    def callback_message(self,ros_marker_array):
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
        self.send_msg(sent_data)

    def callback_pcd(self,ros_pcd : PointCloud2):
        # 靠5G发送
        self.send_pcd(ros_pcd.data)

    def callback_odometry(self,ros_tf):
        global ifm_send
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
        self.send_odm(sent_data)

    def callback_img(self,ros_img : Image):
        # ifm.send_img(ros_img.data)
        img = np.ndarray(shape=(480, 640, 3), dtype=np.dtype("uint8"), buffer=ros_img.data)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640//2, 480//2), interpolation = cv2.INTER_AREA)
        # print("recieve img")
        # cv2.imshow('Image_'+str(self.robot_id), img)
        # cv2.waitKey(1)
        _, jpeg = cv2.imencode('.jpg', img)
        data = jpeg.tobytes()
        # print('send img', len(data))
        self.send_img(data)
        #send msg to edge by tcp/ip
       
    def callback_pcd_kf(self,submap : SubMap):
        pose = np.array([submap.pose.position.x, submap.pose.position.y,submap.pose.position.z,
                    submap.pose.orientation.x,submap.pose.orientation.y,submap.pose.orientation.z,
                    submap.pose.orientation.w], dtype='float64')
        #float64 is 4bytes
        #0-27 is pose
        byte_pose = pose.tobytes()
        len_pc_1 = np.array([len(submap.keyframePC.data)],dtype='int32')
        print('send kf_pcd_len'+str(len_pc_1[0]))
        byte_len_pc_1 = len_pc_1.tobytes()
        
        # # len_pc_2 = len(submap.submap.data)
        # byte_len_pc_2 = len_pc_2.to_bytes()

        sent_data = byte_pose+byte_len_pc_1+submap.keyframePC.data+submap.submap.data
        self.send_pcd_kf(sent_data) 
       
    def send_msg(self, message):
        self.send(message, 'msg')
    
    def send_odm(self, message):
        self.send(message, 'odm')

    def send_pcd(self, message):
        self.send(message, 'pcd')

    def send_img(self, message):
        self.send(message, 'img')

    def send_pcd_kf(self,message):
        self.send(message,'pcd_kf')

    def __init__(self,config,robot_id) -> None:
        
        
        self.tf_sub = rospy.Subscriber('/robot_'+str(robot_id)+'/tf', TFMessage, self.callback_odometry)
        
        self.pcd_kf_sub = rospy.Subscriber('/robot_'+str(robot_id)+'/submap', SubMap, self.callback_pcd_kf)
        #self.pc_sub = rospy.Subscriber('/robot_'+str(robot_id)+'/point_cloud2', PointCloud2, self.callback_pcd)
        #self.img_sub = rospy.Subscriber('/robot_'+str(robot_id)+'/stereo_color/right/image_color', Image, self.callback_img)
        #self.ob_sub = rospy.Subscriber('/robot_'+str(robot_id)+'/detection/lidar_detector/objects_markers', MarkerArray, self.callback_message)
        super().__init__(config,robot_id)


class ClientRecv(Informer):
    #recieve msg from edge by tcp/ip
    def msg_recv(self):
        self.recv('msg', self.parse_msg)

    def pcd_recv(self):
        self.recv('pcd', self.parse_sem_pcd)

    def img_recv(self):
        self.recv('img', self.parse_img)

    def odm_recv(self):
        self.recv('odm', self.parse_odm)
    
    def pcd_kf_recv(self):
        self.recv('pcd_kf',self.parse_pcd_kf)

    def parse_odm(self,message, robot_id):
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
        
        self.tf_pub.publish(ros_tf)

    def parse_img(self,message, robot_id):
        # print('img', len(message))
        nparr = np.frombuffer(message, np.uint8)
        img_data = cv2.imdecode(nparr,  cv2.IMREAD_COLOR)

        print('-------recv from '+str(robot_id)+'--------')
        #cv2.imshow('Image_'+str(self.robot_id), img_data)
        cv2.waitKey(1)

        # img_data = cv2.cvtColor(img_data, cv2.COLOR_BGR2RGB)
        img_data = np.reshape(img_data, ((640//2)*(480//2)*3, 1))
        img = Image()
        img.data = img_data.tobytes()
        img.height = 480//2
        img.width = 640//2
        img.encoding = "bgr8"
        img.is_bigendian = 0
        img.step = 6144
      
        self.img_pub.publish(img)


    def parse_msg(self,message, robot_id):
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
        
        self.marker_pub.publish(marker_list)

    def parse_sem_pcd(self,message, robot_id):
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

        self.pcd_pub.publish(pcd)
    
    def parse_pcd_kf(self,message,robot_id):
        submap_recv = SubMap()
        
        pose_data = message[0:56]

        #pcd keyframe
        bytes_len_pcd_1 = message[56:60]
        len_array = np.frombuffer(bytes_len_pcd_1,dtype='int32',count=1)
        recv_len_pcd_1 = len_array[0]
        pcd_data = message[60:60+recv_len_pcd_1]
        print('recv kf_pcd_len'+str(recv_len_pcd_1))
        #pcd submap
        # bytes_len_pcd_2 = message[60+len_pcd_1:60+len_pcd_1+4]
        # len_pcd_2 = int.from_bytes(bytes_len_pcd_2)
        submap_data = message[60+recv_len_pcd_1:]

        pose_array = np.frombuffer(pose_data,dtype='float64',count=7)
        #print('send kf_pcd_len'+str(recv_len_pcd_1))
        
        

        #pose
        submap_recv.pose.position.x = pose_array[0] 
        submap_recv.pose.position.y = pose_array[1]
        submap_recv.pose.position.z = pose_array[2]
        submap_recv.pose.orientation.x = pose_array[3]
        submap_recv.pose.orientation.y = pose_array[4]
        submap_recv.pose.orientation.z = pose_array[5]
        submap_recv.pose.orientation.w = pose_array[6]

        #keyframe
        pcd = PointCloud2()
        pcd.header = Header()
        # pcd.header.stamp = rospy.Time.now()
        pcd.header.frame_id = 'robot_'+str(self.robot_id)+'/os_sensor'
        pcd.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]
        pcd.data = pcd_data
        pcd.point_step = 16
        pcd.width = len(pcd.data)//pcd.point_step
        pcd.height = 1
        pcd.row_step = len(pcd.data)
        pcd.is_bigendian = False
        pcd.is_dense = True
        submap_recv.keyframePC = pcd


        pcd = PointCloud2()
        pcd.header = Header()
        # pcd.header.stamp = rospy.Time.now()
        pcd.header.frame_id = 'robot_'+str(self.robot_id)+'/odom'
        pcd.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]
        pcd.data = submap_data
        pcd.point_step = 16
        pcd.width = len(pcd.data)//pcd.point_step
        pcd.height = 1
        pcd.row_step = len(pcd.data)
        pcd.is_bigendian = False
        pcd.is_dense = True
        submap_recv.submap = pcd
        self.pcd_kf_pub.publish(submap_recv)

    def __init__(self,config,robot_id) -> None:
       
        self.pcd_kf_pub  = rospy.Publisher('/robot_'+str(robot_id)+'/submap', SubMap, queue_size=0)
        self.tf_pub = rospy.Publisher('/robot_'+str(robot_id)+'/tf', TFMessage, queue_size=0)
        #self.pcd_pub = rospy.Publisher('/robot_'+str(robot_id)+'/lidar_center/velodyne_points2', PointCloud2, queue_size=0)
        #self.img_pub = rospy.Publisher('/robot_'+str(robot_id)+'/stereo_color/right/image_color2_recv', Image, queue_size=0)
        #self.marker_pub = rospy.Publisher('/robot_'+str(robot_id)+'/detection/lidar_detector/objects_markers2', MarkerArray, queue_size=0)
        #self.tf_pub = rospy.Publisher('/robot_'+str(robot_id)+'/tf2', TFMessage, queue_size=0)
        
        
        super().__init__(config,robot_id)

