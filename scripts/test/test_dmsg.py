import rospy
from dislam_msgs.msg import SubMap

import numpy as np

def callback_submap(submap):
    print("x is"+str(submap.pose.position.x))
    print("y is"+str(submap.pose.position.y))
    print("z is"+str(submap.pose.position.z))
    print(submap.pose.orientation.x,submap.pose.orientation.y,
          submap.pose.orientation.z,submap.pose.orientation.w)

          
    pose = np.array([submap.pose.position.x, submap.pose.position.y,submap.pose.position.z,
                  submap.pose.orientation.x,submap.pose.orientation.y,submap.pose.orientation.z,
                  submap.pose.orientation.w], dtype='float64')
    byte_pose = pose.tobytes()
    print(len(byte_pose))
    message = byte_pose+submap.keyframePC.data
    byte_kf = byte_pose+submap.keyframePC.data
    print(type(byte_kf))
    # print(byte_x))
    print("pcd height"+str(submap.keyframePC.height))
    print("pcd width"+str(submap.keyframePC.height))


    
  
    pose_data = ''
    pcd_data = ''
    for i in range(len(message)):
        if(i<56):
            pose_data=pose_data+message[i]
        else:
            pcd_data=pcd_data+message[i]
    pose_array = np.frombuffer(pose_data,dtype='float64',count=7)
    for i in range(7):
        print("  %f  ",pose_array[i])

    

if __name__ == '__main__':
    rospy.init_node('vehicle_5g_transfer', anonymous=True)
    robot_id = 1
    tf_sub = rospy.Subscriber('/robot_'+str(robot_id)+'/submap', SubMap, callback_submap)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()