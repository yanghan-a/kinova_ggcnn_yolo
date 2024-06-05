#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import math
import sys
import tf
import numpy as np
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *
from std_msgs.msg import Float32MultiArray
import tf.transformations as transformations
from geometry_msgs.msg import Pose ,PoseStamped
from std_msgs.msg import Header

class Public_Object_Pose:
    def __init__(self):
        rospy.init_node('pub_object_pose')
        self.object_pose_pub = rospy.Publisher("/PoseStamped",PoseStamped,queue_size=1)
        self.listener = tf.TransformListener()
        self.source_frame = "base_link"  # 源坐标系
        self.target_frame = "camera_color_optical_frame"  # 目标坐标系
        # target_frame = "camera_aligned_depth_to_color_frame"  # 目标坐标系

        self.sub_ggcnn = rospy.Subscriber('ggcnn/out/command',Float32MultiArray,self.ggcnn_grasp_callback)
        print('test')
    def rotation_matrix_to_euler_angles(self,R):
        """
        Convert rotation matrix to Euler angles (yaw, pitch, roll).
        Assumes the rotation matrix represents rotations in ZYX order.
        """
        # Extract rotation angles
        theta_x = np.arctan2(R[2, 1], R[2, 2])
        theta_y = np.arctan2(-R[2, 0], math.sqrt(R[2,1]**2+R[2,2]**2))
        theta_z = np.arctan2(R[1, 0], R[0, 0])
        return np.array([theta_x, theta_y, theta_z])  
    def ggcnn_grasp_callback(self,msg):
        print('test')
        self.listener.waitForTransform(self.source_frame,self.target_frame, rospy.Time(), rospy.Duration(10.0))
        
        # 获取两个坐标系之间的变换
        (trans, rot) = self.listener.lookupTransform(self.source_frame,self.target_frame,  rospy.Time(0))
        rotation = np.array(rot)
        transform_matrix = tf.transformations.quaternion_matrix(rotation)
        transform_matrix[:3, 3] = trans
        self.camera_world = transform_matrix
        print("camera2base_link",self.camera_world)
        x = msg.data[0]
        y = msg.data[1]
        z = msg.data[2]
        angle = msg.data[3]#-np.pi/2
        print("angle:",angle)
        #物体相对于相机变换矩阵,注意这里x轴的向量与一般的不同
        object2camera =    [[np.cos(angle),   np.sin(angle),  0, x],
                            [-np.sin(angle),  np.cos(angle),  0, y],
                            [ 0,              0,               1, z],
                            [ 0,              0,               0, 1]]

        object2base_link = np.dot(self.camera_world,object2camera)
        rotation = object2base_link[:3,:3]
        theta_x,theta_y,theta_z = self.rotation_matrix_to_euler_angles(object2base_link)

        print('x:',object2base_link[0][3])
        print('y:',object2base_link[1][3])
        print('z:',object2base_link[2][3])

        # 创建一个 PoseStamped 消息对象
        pose_stamped_msg = PoseStamped()

        # 设置 Header，包括时间戳和坐标系信息
        pose_stamped_msg.header = Header()
        pose_stamped_msg.header.stamp = rospy.Time.now()
        pose_stamped_msg.header.frame_id = "base_link"  # 坐标系为 base_link

        quaternion = transformations.quaternion_from_euler(theta_x, theta_y, theta_z)
        pose_object = Pose()
        pose_object.position.x = object2base_link[0][3]
        pose_object.position.y = object2base_link[1][3]
        pose_object.position.z = object2base_link[2][3]

        pose_object.orientation.x = quaternion[0]
        pose_object.orientation.y = quaternion[1]
        pose_object.orientation.z = quaternion[2]
        pose_object.orientation.w = quaternion[3]

        pose_stamped_msg.pose = pose_object
        self.object_pose_pub.publish(pose_stamped_msg)     

if __name__ == "__main__":
    ex = Public_Object_Pose()
    rospy.spin()
