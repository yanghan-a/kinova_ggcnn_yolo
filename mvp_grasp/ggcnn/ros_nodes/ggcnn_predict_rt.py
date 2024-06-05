#! /usr/bin/env python

import sys
import time
from os import path
import rospy

import numpy as np

import tensorflow as tf

import cv2
import scipy.ndimage as ndimage
from skimage.draw import ellipse
from skimage.feature import peak_local_max

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
sys.path.append("/home/yh/ggcnn_mvp/src/mvp_grasp/ggcnn/src")
sys.path.append("/home/yh/ggcnn_mvp/src/mvp_grasp/dougsm_helpers/src")



# import os
# import sys

# print('当前工作路径: ', os.getcwd())
# print('导包路径为: ')

# for p in sys.path:
#     print(p)


from ggcnn.ggcnn import predict
from dougsm_helpers.timeit import TimeIt

bridge = CvBridge()

rospy.init_node('ggcnn_detection')

# Output publishers.
grasp_pub = rospy.Publisher('ggcnn/img/grasp', Image, queue_size=1)
grasp_plain_pub = rospy.Publisher('ggcnn/img/grasp_plain', Image, queue_size=1)
depth_pub = rospy.Publisher('ggcnn/img/depth', Image, queue_size=1)
ang_pub = rospy.Publisher('ggcnn/img/ang', Image, queue_size=1)
cmd_pub = rospy.Publisher('ggcnn/out/command', Float32MultiArray, queue_size=1)


# Initialise some globals.
prev_mp = np.array([150, 150])

# Get the camera parameters
# camera_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
# camera_info_msg = rospy.wait_for_message('/camera/aligned_depth_to_color/camera_info', CameraInfo)
# [ 640x480  p[321.179 237.103]  f[605.925 605.553]  Inverse Brown Conrady [0 0 0 0 0] ]
# K = camera_info_msg.K
# fx = K[0]
# cx = K[2]
# fy = K[4]
# cy = K[5]
fx = 605.925
cx = 321.179
fy = 605.553
cy = 237.103

recognize_image = None
def recognize_callback(recognize_message):
    global recognize_image
    recognize_image = bridge.imgmsg_to_cv2(recognize_message)
    # print('有数据')







def depth_callback(depth_message):
    # print('收到深度')
    global recognize_image 
    if recognize_image is not None:
        global prev_mp
        global fx, cx, fy, cy

        with TimeIt('Predict'):
            depth = bridge.imgmsg_to_cv2(depth_message)
            #把深度图像转化为m作单位
            depth = depth / 1000.0
            #  Crop a square out of the middle of the depth and resize it to 300*300
            #注意这里crop_size和out_size的区别,这里就看着改，达到比较好的效果
            # crop_size = 300
            # crop_offset = 0
            # out_size = 300

            #这是仿真中第二个相机位置的参数
            # crop_size = 300
            # crop_x_offset = -100
            # crop_y_offset = 0
            # out_size = 300

            # crop是初次裁减的，out是最终放进模型的
            crop_size = 480
            crop_x_offset = 0
            crop_y_offset = 0
            out_size = 300
            
            # depth_crop输出为300*300
            points_out, ang_out, width_out, depth_crop = predict(depth, crop_size=crop_size, out_size=out_size, crop_y_offset=crop_y_offset,crop_x_offset=crop_x_offset, filters=(2.0, 2.0, 2.0))


            points_out[recognize_image==0] = 0
            recognize_image = None
        with TimeIt('Calculate Depth'):
            # Figure out roughly the depth in mm of the part between the grippers for collision avoidance.
            depth_center = depth_crop[100:141, 130:171].flatten()
            depth_center.sort()
            depth_center = depth_center[:10].mean() * 1000.0

        with TimeIt('Control'):
            # Calculate the best pose from the camera intrinsics.
            maxes = None
            ALWAYS_MAX = True  # Use ALWAYS_MAX = True for the open-loop solution.

            if ALWAYS_MAX:
                # Track the global max.
                max_pixel = np.array(np.unravel_index(np.argmax(points_out), points_out.shape))
                prev_mp = max_pixel.astype(np.int)
                # print("全局最大值位置：",prev_mp)
                # print("全局最大值：",points_out[max_pixel[0]][max_pixel[1]])
            else:
                # Calculate a set of local maxes.  Choose the one that is closes to the previous one.
                maxes = peak_local_max(points_out, min_distance=10, threshold_abs=0.1, num_peaks=3)
                print(maxes)
                if maxes.shape[0] == 0:
                    rospy.logerr('No Local Maxes')
                    return
                max_pixel = maxes[np.argmin(np.linalg.norm(maxes - prev_mp, axis=1))]

                # Keep a global copy for next iteration.
                prev_mp = (max_pixel * 0.25 + prev_mp * 0.75).astype(np.int)

            ang = ang_out[max_pixel[0], max_pixel[1]]
            width = width_out[max_pixel[0], max_pixel[1]]

            # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
            max_pixel = ((np.array(max_pixel) / out_size * crop_size) + np.array([(480 - crop_size)//2 - crop_y_offset, (640 - crop_size) // 2- crop_x_offset]))
            max_pixel = np.round(max_pixel).astype(np.int)

            point_depth = depth[max_pixel[0], max_pixel[1]]
            print('point_depth',point_depth)
            # Compute the actual position.
            x = (max_pixel[1] - cx)/(fx) * point_depth
            y = (max_pixel[0] - cy)/(fy) * point_depth
            z = point_depth

            if np.isnan(z):
                return

        with TimeIt('Draw'):
            # Draw grasp markers on the points_out and publish it. (for visualisation)
            grasp_img = cv2.applyColorMap((points_out * 255).astype(np.uint8), cv2.COLORMAP_JET)
            grasp_img_plain = grasp_img.copy()

            # rr, cc = circle(prev_mp[0], prev_mp[1], 5)
            rr, cc = ellipse(prev_mp[0], prev_mp[1], 5, 5)
            grasp_img[rr, cc, 0] = 0
            grasp_img[rr, cc, 1] = 255
            grasp_img[rr, cc, 2] = 0

        with TimeIt('Publish'):
            # Publish the output images (not used for control, only visualisation)
            grasp_img = bridge.cv2_to_imgmsg(grasp_img, 'bgr8')
            grasp_img.header = depth_message.header
            grasp_pub.publish(grasp_img)

            grasp_img_plain = bridge.cv2_to_imgmsg(grasp_img_plain, 'bgr8')
            grasp_img_plain.header = depth_message.header
            grasp_plain_pub.publish(grasp_img_plain)

            #将32FC1转为16UC1
            #depth_crop = cv2.convertScaleAbs(depth_crop, alpha=(65535.0))
            depth_crop = depth_crop
            depth_crop = cv2.normalize(depth_crop, None, 0, 255, cv2.NORM_MINMAX)
            depth_crop = cv2.convertScaleAbs(depth_crop)
            depth_crop = bridge.cv2_to_imgmsg(depth_crop,encoding = 'passthrough')
            depth_crop.header = depth_message.header
            depth_pub.publish(depth_crop)

            ang_out = cv2.applyColorMap((ang_out * 255).astype(np.uint8), cv2.COLORMAP_JET)
            ang_out = bridge.cv2_to_imgmsg(ang_out,encoding =  'bgr8')
            ang_out.header = depth_message.header

            ang_pub.publish(ang_out)

            #if x !=0 and y!=0 and z!=0:
            if z!=0:
            # Output the best grasp pose relative to camera.
                cmd_msg = Float32MultiArray()
                cmd_msg.data = [x, y, z, ang, width, depth_center]
                cmd_pub.publish(cmd_msg)
                # print('x:',x)
                # print('y:',y)
                # print('z:',z)
                # print('ang:',ang)
                print('width:',width)
                # print('depth_center:',depth_center)


# depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback, queue_size=1)
# depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback, queue_size=1)

depth_sub = rospy.Subscriber('camera/depth/aliged_depth_post_processing_image', Image, depth_callback, queue_size=1)

# depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw/compressed', Image, depth_callback, queue_size=1)
# depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1)

recognize_sub = rospy.Subscriber('yolov5/image_out_masked',Image, recognize_callback, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()
