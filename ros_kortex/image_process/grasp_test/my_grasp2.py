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

# 定义了一个比较好的抓取位置，对应的欧拉角如下
# [[ 0.04371656 -0.99804331 -0.04470353 -0.04225469]
#  [-0.99898416 -0.04318054 -0.01288719  0.36617767]
#  [ 0.01093165  0.0452215  -0.99891717  0.20315396]
#  [ 0.          0.          0.          1.        ]]
#位置XYZ：[ -0.04225469 0.36617767 0.20315396]
#欧拉角XYZ： [ 3.09635302 -0.01093187 -1.52706322]

#角度制 [177.40796001  -0.62634988 -87.49427727]

class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        try:
            
            rospy.init_node('example_cartesian_poses_with_notifications_python')
            self.object_pose_pub = rospy.Publisher("/PoseStamped",PoseStamped,queue_size=10)
            self.listener = tf.TransformListener()
            self.source_frame = "base_link"  # 源坐标系
            self.target_frame = "camera_color_optical_frame"  # 目标坐标系
            self.x = None
            self.y = None
            self.z = None
            self.theta_x = None
            self.theta_y = None
            self.theta_z = None
            # target_frame = "camera_aligned_depth_to_color_frame"  # 目标坐标系
            # 等待变换可用
            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            self.start_grasp = False
            #订阅ggcnn中发来的点的位置
            print('类真的在声明')
            self.sub_ggcnn = rospy.Subscriber('ggcnn/out/command',Float32MultiArray,self.ggcnn_grasp_callback)
            self.sub_grasp_pose = rospy.Subscriber('/PoseStamped',PoseStamped,self.start_grasp_pose)
            
            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", True)
            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            #用来记录机器人不同时刻的action状态
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            #读取关节将要运行的位置
            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            #执行关节运动
            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            #猜测是控制夹爪的服务
            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            #激活发布机器人action状态的service
            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
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
    def start_grasp_pose(self,msg):
        if self.start_grasp == True and self.x is not None:
            self.start_grasp = False
            # if self.is_gripper_present:
            #     self.example_send_gripper_command(0.3)
            # else:
            #     rospy.logwarn("No gripper is present on the arm.")
            # self.start_grasp = False
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.1 # m/s
            my_cartesian_speed.orientation = 40  # deg/s
            req = ExecuteActionRequest()

            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
            req.input.name = "move"
            req.input.handle.action_type = ActionType.REACH_POSE
            req.input.handle.identifier = 1001
            local_z = self.z
            my_constrained_pose.target_pose.x = self.x
            
            my_constrained_pose.target_pose.y = self.y
            my_constrained_pose.target_pose.z = 0.15
            my_constrained_pose.target_pose.theta_x = np.degrees(self.theta_x)
            my_constrained_pose.target_pose.theta_y = np.degrees(self.theta_y)
            my_constrained_pose.target_pose.theta_z = np.degrees(self.theta_z)
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)

            rospy.loginfo("Sending move...")
            self.last_action_notif_type = None
            self.execute_action(req)
            self.wait_for_action_end_or_abort()
            
            my_constrained_pose.target_pose.z = local_z
            my_cartesian_speed.translation = 0.04 # m/s
            self.last_action_notif_type = None
            self.execute_action(req)
            self.wait_for_action_end_or_abort()
            rospy.sleep(1.0)
            # Example of gripper command
            # Let's close the gripper at 50%
            if self.is_gripper_present:
                self.example_send_gripper_command(1.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")
            

            rospy.sleep(2)
            my_cartesian_speed.translation = 0.20 # m/s
            self.last_action_notif_type = None
            my_constrained_pose.target_pose.z = 0.2071259
            self.execute_action(req)
            self.wait_for_action_end_or_abort()
            

            my_constrained_pose.target_pose.x = 0.294
            my_constrained_pose.target_pose.y = -0.33991108
            my_constrained_pose.target_pose.z = local_z+0.02
            my_constrained_pose.target_pose.theta_x = -180   
            my_constrained_pose.target_pose.theta_y =  0.56  
            my_constrained_pose.target_pose.theta_z = -135
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            rospy.loginfo("Sending move...")
            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send move")
                # success = False
            else:
                rospy.loginfo("Waiting for move to finish...")

            self.wait_for_action_end_or_abort()
            # Example of gripper command
            # Let's close the gripper at 50%
            if self.is_gripper_present:
                self.example_send_gripper_command(0.3)
            else:
                rospy.logwarn("No gripper is present on the arm.")
            # 物体抓取完到达的姿态
            # [[-0.59992483 -0.31689019  0.7346229   0.38795525]
            # [-0.77218053  0.46961003 -0.42802295 -0.32991108]
            # [-0.20935001 -0.8240431  -0.52642715  0.21871259]
            # [ 0.          0.          0.          1.        ]]
            # self.example_home_the_robot()
            rospy.sleep(1)

            my_constrained_pose.target_pose.x =  2.94882548e-01
            my_constrained_pose.target_pose.y = -7.09835067e-04 
            my_constrained_pose.target_pose.z = 1.98944127e-01+0.04
            my_constrained_pose.target_pose.theta_x = 180.40796001 
            my_constrained_pose.target_pose.theta_y = -0.62634988 
            my_constrained_pose.target_pose.theta_z = -90.49427727
            self.last_action_notif_type = None
            self.execute_action(req)
            self.wait_for_action_end_or_abort()
            rospy.sleep(5)
            self.start_grasp = True
            self.x = None
    def ggcnn_grasp_callback(self,msg):
        self.listener.waitForTransform(self.source_frame,self.target_frame, rospy.Time(), rospy.Duration(5.0))
        
        # 获取两个坐标系之间的变换
        (trans, rot) = self.listener.lookupTransform(self.source_frame,self.target_frame,  rospy.Time(0))
        # 将变换转换为变换矩阵
        rotation = np.array(rot)
        transform_matrix = tf.transformations.quaternion_matrix(rotation)
        transform_matrix[:3, 3] = trans
        self.camera_world = transform_matrix

        x = msg.data[0]
        y = msg.data[1]
        z = msg.data[2]
        angle = msg.data[3]#-np.pi/2
        #物体相对于相机变换矩阵
        object2camera = [[np.cos(angle),   np.sin(angle),  0, x],
                            [-np.sin(angle),  np.cos(angle),  0, y],
                            [ 0,            0,               1, z],
                            [ 0,            0,               0, 1]]
        
        object2base_link = np.dot(self.camera_world,object2camera)
        rotation = object2base_link[:3,:3]
        theta_x,theta_y,theta_z = self.rotation_matrix_to_euler_angles(object2base_link)

        self.x = object2base_link[0][3]+0.01
        self.y = object2base_link[1][3]-0.018
        self.z = object2base_link[2][3]+0.02
        self.theta_x = theta_x
        self.theta_y = theta_y
        self.theta_z = theta_z


        # 创建一个 PoseStamped 消息对象
        pose_stamped_msg = PoseStamped()

        # 设置 Header，包括时间戳和坐标系信息
        pose_stamped_msg.header = Header()
        pose_stamped_msg.header.stamp = rospy.Time.now()
        pose_stamped_msg.header.frame_id = "base_link"  # 坐标系为 base_link

        quaternion = transformations.quaternion_from_euler(theta_x, theta_y, theta_z)
        pose_object = Pose()
        pose_object.position.x = self.x
        pose_object.position.y = self.y
        pose_object.position.z = self.z

        pose_object.orientation.x = quaternion[0]
        pose_object.orientation.y = quaternion[1]
        pose_object.orientation.z = quaternion[2]
        pose_object.orientation.w = quaternion[3]

        pose_stamped_msg.pose = pose_object
        self.object_pose_pub.publish(pose_stamped_msg)


    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:

        #这里主要是定义ReadActionRequest()类型服务，再res = self.read_action(req)发送服务请求
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service

            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                #调用执行服务self.execute_action(req)开始执行
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    #我的理解是调用self.activate_publishing_of_action_notification(req)服务，激活发布ActionNotification类型话题
    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True
    
    #调用self.send_gripper_command(req)来请求服务
    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:

            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Start the example from the Home position
            # success &= self.example_home_the_robot()
            #*******************************************************************************
            # Example of gripper command
            # Let's fully open the gripper
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.3)
            else:
                rospy.logwarn("No gripper is present on the arm.")
            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.example_subscribe_to_a_robot_notification()

            

            # #*******************************************************************************
            # # Prepare and send pose 1
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.2 # m/s
            my_cartesian_speed.orientation = 30  # deg/s

            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

            my_constrained_pose.target_pose.x =  2.94882548e-01
            my_constrained_pose.target_pose.y = -7.09835067e-04 
            my_constrained_pose.target_pose.z = 1.98944127e-01+0.04
            my_constrained_pose.target_pose.theta_x = 180.40796001 
            my_constrained_pose.target_pose.theta_y = -0.62634988 
            my_constrained_pose.target_pose.theta_z = -90.49427727

            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "pose1"
            req.input.handle.action_type = ActionType.REACH_POSE
            req.input.handle.identifier = 1001

            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send pose prepare")
                success = False
            else:
                rospy.loginfo("Waiting for pose prepare to finish...")

            self.wait_for_action_end_or_abort()

            rospy.sleep(5)
            self.start_grasp = True
            

            success &= self.all_notifs_succeeded
        
        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")
        rospy.spin()

if __name__ == "__main__":
    print('类的声明')
    ex = ExampleCartesianActionsWithNotifications()
    print('类已声明')
    ex.main()
