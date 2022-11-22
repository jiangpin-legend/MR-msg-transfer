import rospy
from dislam_msgs.msg import SubMap
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header

import numpy as np

def callback_submap(submap):
    #----send----
    print("x is"+str(submap.pose.position.x))
    print("y is"+str(submap.pose.position.y))
    print("z is"+str(submap.pose.position.z))
    print(submap.pose.orientation.x,submap.pose.orientation.y,
          submap.pose.orientation.z,submap.pose.orientation.w)

          
    pose = np.array([submap.pose.position.x, submap.pose.position.y,submap.pose.position.z,
                  submap.pose.orientation.x,submap.pose.orientation.y,submap.pose.orientation.z,
                  submap.pose.orientation.w], dtype='float64')

    byte_pose = pose.tobytes()
    len_pc_1 = np.array([len(submap.keyframePC.data)],dtype='int32')
    print('send kf_pcd_len'+str(len_pc_1[0]))
    byte_len_pc_1 = len_pc_1.tobytes()
    
    # # len_pc_2 = len(submap.submap.data)
    # byte_len_pc_2 = len_pc_2.to_bytes()

    message = byte_pose+byte_len_pc_1+submap.keyframePC.data+submap.submap.data
    #self.send_pcd_kf(sent_data)


    #----recv----
    submap_recv = SubMap()
        
    #pose_data
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
    pcd.header.frame_id = 'lidar_center'
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
    pcd.header.frame_id = 'lidar_center'
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


    
  

    

if __name__ == '__main__':
    rospy.init_node('vehicle_5g_transfer', anonymous=True)
    robot_id = 1
    tf_sub = rospy.Subscriber('/robot_'+str(robot_id)+'/submap', SubMap, callback_submap)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()