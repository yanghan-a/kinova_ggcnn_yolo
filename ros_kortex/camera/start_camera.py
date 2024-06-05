import numpy as np  # fundamental package for scientific computing 科学计算的基本软件包
import pyrealsense2 as rs  # Intel RealSense cross-platform open-source API 英特尔实感跨平台开源API
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import RegionOfInterest
import rospy
import image_processing


# -*- coding: utf-8 -*-
# 依次顺序应用这些过滤器时效果最佳。
# 在更长的范围内，它还有助于使用disparity_transform从深度表示转换为视差形式：
rospy.init_node('depth_post_processing')
color_origin_pub = rospy.Publisher('camera/color/color_origin_image', Image, queue_size=1)
color_crop_pub = rospy.Publisher('camera/color/color_crop_image', Image, queue_size=1)
aligned_depth_origin_pub = rospy.Publisher('camera/depth/aliged_origin_image', Image, queue_size=1)
aligned_depth_post_processing_pub = rospy.Publisher('camera/depth/aliged_depth_post_processing_image', Image, queue_size=1)
camerainfo_pub = rospy.Publisher('/camera/color/camera_info',CameraInfo,queue_size=1)
bridge = CvBridge()

print("Environment Ready")

# Setup: 配置】
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg = pipe.start(config)
device = cfg.get_device()
name = device.get_info(rs.camera_info.name)
print(name)
#获取相机的内参矩阵
profile_depth = cfg.get_stream(rs.stream.depth)
profile_color = cfg.get_stream(rs.stream.color)
intr_depth = profile_depth.as_video_stream_profile().get_intrinsics()
intr_color = profile_color.as_video_stream_profile().get_intrinsics()
print(intr_color)
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)

# 【Skip 5 first frames to give the Auto-Exposure time to adjust 跳过前5帧以设置自动曝光时间】
for x in range(5):
    pipe.wait_for_frames()

while not rospy.is_shutdown():
    frameset = pipe.wait_for_frames()
    # depth_frame = frameset.get_depth_frame()
    # depth_frame = np.asanyarray(depth_frame.get_data())
    # depth_image_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
    # depth_image_normalized = cv2.convertScaleAbs(depth_image_normalized)
    # cv2.imshow('original_depth_image',depth_image_normalized)
    # cv2.waitKey(1)


    aligned_frames = align.process(frameset)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）

    camerainfo = CameraInfo()
    camerainfo.header.seq = 1
    camerainfo.header.stamp = rospy.Time.now()
    camerainfo.header.frame_id = "camera_color_optical_frame"
    camerainfo.height = 480
    camerainfo.width = 640
    camerainfo.distortion_model = "plumb_bob"
    camerainfo.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camerainfo.K = [605.9246215820312, 0.0, 321.1793518066406, 0.0, 605.5530395507812, 237.10296630859375, 0.0, 0.0, 1.0]
    camerainfo.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camerainfo.P = [605.9246215820312, 0.0, 321.1793518066406, 0.0, 0.0, 605.5530395507812, 237.10296630859375, 0.0, 0.0, 0.0, 1.0, 0.0]

    camerainfo.binning_x = 0
    camerainfo.binning_y = 0
    roi = RegionOfInterest()
    camerainfo.roi.x_offset = 0
    camerainfo.roi.y_offset = 0
    camerainfo.roi.height = 0
    camerainfo.roi.width = 0
    camerainfo.roi.do_rectify = False
    camerainfo_pub.publish(camerainfo)


    crop_size = 480
    crop_x_offset = 0
    crop_y_offset = 0
    imh = 480
    imw = 640
    #这里发布彩色图
    # color_frame_origin = cv2.imread(color_frame)
    color_frame_origin = np.asanyarray(color_frame.get_data())
    color_frame_crop = color_frame_origin[(imh - crop_size) // 2 - crop_y_offset:(imh - crop_size) // 2 + crop_size - crop_y_offset,
                            (imw - crop_size) // 2- crop_x_offset:(imw - crop_size) // 2 + crop_size- crop_x_offset]
    
    # cv2.imshow('color',color_frame_origin)
    # cv2.waitKey(1)
    # cv2.imshow('color_crop',color_frame_crop)
    # cv2.waitKey(1)
    color_frame_origin = bridge.cv2_to_imgmsg(color_frame_origin, encoding="bgr8")
    color_frame_origin.header.seq = 111
    color_frame_origin.header.stamp = rospy.Time.now()
    color_frame_origin.header.frame_id = "camera_color_optical_frame"
    color_origin_pub.publish(color_frame_origin)


    color_frame_crop = bridge.cv2_to_imgmsg(color_frame_crop, encoding="bgr8")
    color_frame_crop.header.seq = 111
    color_frame_crop.header.stamp = rospy.Time.now()
    color_frame_crop.header.frame_id = "camera_color_optical_frame"
    color_crop_pub.publish(color_frame_crop)


    #这里发布对齐后的深度图
    aligned_depth_frame_origin = np.asanyarray(aligned_depth_frame.get_data())
    depth_image_normalized = cv2.normalize(aligned_depth_frame_origin, None, 0, 255, cv2.NORM_MINMAX)
    depth_image_normalized = cv2.convertScaleAbs(depth_image_normalized)
    # cv2.imshow('depth',depth_image_normalized)
    # cv2.waitKey(1)
    aligned_depth_frame_origin = bridge.cv2_to_imgmsg(aligned_depth_frame_origin, encoding="passthrough")
    aligned_depth_frame_origin.header.seq = 111
    aligned_depth_frame_origin.header.stamp = rospy.Time.now()
    aligned_depth_frame_origin.header.frame_id = "camera_color_optical_frame"
    aligned_depth_origin_pub.publish(aligned_depth_frame_origin)



    #这里处理对齐后的深度数据
    aligned_depth_frame = image_processing.image_processing(aligned_depth_frame)

    #这里发布处理后的深度图
    aligned_depth_frame = np.asanyarray(aligned_depth_frame.get_data())
    depth_image_normalized = cv2.normalize(aligned_depth_frame, None, 0, 255, cv2.NORM_MINMAX)
    depth_image_normalized = cv2.convertScaleAbs(depth_image_normalized)
    # cv2.imshow('processed_depth',depth_image_normalized)
    # cv2.waitKey(1)
    aligned_depth_frame = bridge.cv2_to_imgmsg(aligned_depth_frame, encoding="passthrough")
    aligned_depth_frame.header.seq = 111
    aligned_depth_frame.header.stamp = rospy.Time.now()
    aligned_depth_frame.header.frame_id = "camera_color_optical_frame"
    aligned_depth_post_processing_pub.publish(aligned_depth_frame)
    
    

# pipe.stop()
# print("Frames Captured")



# rospy.init_node('depth_post_processing')
# bridge = CvBridge()
# depth_normalized_pub = rospy.Publisher('ggcnn/out/depth_normalized', Image, queue_size=1)
# depth_frame_data = None
# def depth_post_processing_callback(depth_message):
#     global depth_frame_data
#     try:
#         # Convert ROS Image message to OpenCV image
#         depth_frame_data = bridge.imgmsg_to_cv2(depth_message)

#         depth_image_normalized = cv2.normalize(depth_frame_data, None, 0, 255, cv2.NORM_MINMAX)
#         depth_image_normalized = cv2.convertScaleAbs(depth_image_normalized)
#         cv2.imshow('ss',depth_image_normalized)
#         cv2.waitKey(1)
#         depth_normalized = bridge.cv2_to_imgmsg(depth_image_normalized)
#         depth_normalized.header = depth_message.header
#         depth_normalized_pub.publish(depth_normalized)
#     except CvBridgeError as e:
#         print(e)

# depth_post_processing_pub = rospy.Publisher('camera/depth/depth_post_processing', Image, queue_size=1)
# depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_post_processing_callback, queue_size=1)

# while not rospy.is_shutdown():
#     rospy.spin()