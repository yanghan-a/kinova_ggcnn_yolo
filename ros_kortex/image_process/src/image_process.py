import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

from geometry_msgs.msg import Pose
import tf
from tf.transformations import euler_from_quaternion

class ImageProcess:
    def __init__(self):
        rospy.init_node("image_process")
        self.bridge = CvBridge()
        
        self.depth_image = None
        self.extracted_image = None

        # # 订阅相机信息话题
        # self.color_depth_info_topic = "camera/depth/camera_info"
        # self.camera_depth_info = rospy.Subscriber('/camera_info', CameraInfo, self.camera_info_callback)
        # 订阅彩色图像话题
        self.color_image_topic = "camera/color/image_raw"
        self.color_sub = rospy.Subscriber(self.color_image_topic, Image, self.color_callback)
        # subscribe depth image
        # self.depth_image_topic = "/camera/depth/image_rect_raw"
        self.depth_image_topic ="/camera/depth/aliged_depth_post_processing_image"

        # self.depth_image_topic = "/camera/aligned_depth_to_color/image_raw"
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)

        self.configuration_pub = rospy.Publisher('/my_pose', Pose, queue_size=10)
        self.depth_file = '/home/yh/kinova_robot/src/ros_kortex/image_process/figures_and_data/depth_msg.txt'
        self.depth_file_processed = '/home/yh/kinova_robot/src/ros_kortex/image_process/figures_and_data/depth_file_processed.txt'

        self.color_file = '/home/yh/kinova_robot/src/ros_kortex/image_process/figures_and_data/color_msg.txt'
        

    def color_callback(self, image_msg):
        # 将ROS图像消息转换为OpenCV图像
        self.cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        with open(self.color_file, 'w') as file:
                file.write(str(image_msg))
        # # 转换图像颜色空间为HSV
        # hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        # # 定义颜色范围（例：提取红色）
        # lower_red = np.array([0, 100, 100])
        # upper_red = np.array([10, 255, 255])
        # # 创建掩膜（只保留颜色范围内的像素）
        # mask = cv2.inRange(hsv_image, lower_red, upper_red)
        # # 应用掩膜到原始图像
        # self.extracted_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
        # # 寻找轮廓
        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # # 寻找最大轮廓
        # if len(contours) > 0:
        #     max_contour = max(contours, key=cv2.contourArea)
        #     # 计算轮廓的中心坐标
        #     M = cv2.moments(max_contour)
        #     self.cX = int(M["m10"] / M["m00"])
        #     self.cY = int(M["m01"] / M["m00"])
        #     # 在图像上绘制中心点
        #     #cv2.circle(self.extracted_image, (cX, cY), 5, (0, 255, 0), -1)
        # # 保存提取的图像到文件
        cv2.imwrite("/home/yh/kinova_robot/src/ros_kortex/image_process/figures_and_data/color_image.png", self.cv_image)

    def depth_callback(self, depth_msg):

        # my data is encoded as 16UC1 but not 32FC1, each pixel needs only 2 bytes to express
        n = 2

        try:
            # # the first method
            with open(self.depth_file, 'w') as file:
                file.write(str(depth_msg))
            print("Length of depth_msg data: ", len(depth_msg.data)) # 614400
            # 将深度图像转换为OpenCV格式
            #depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg)
            #depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2RGB)
            print("the size of depth_image",np.size(depth_image))
            # 将深度值从毫米转换为米
            depth_image_m = depth_image.astype(float) / 1000.0
            # 缩放深度值范围到0到255之间
            depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)

            # 将深度图像的数据类型转换为uint8
            depth_image_uint8 = cv2.convertScaleAbs(depth_image_normalized)
            cv2.imshow('original_depth_image',depth_image_uint8)
            cv2.waitKey(1)
            # 将深度图像保存为PNG文件，只有把它归到0-255才能显示出来
            cv2.imwrite('/home/yh/kinova_robot/src/ros_kortex/image_process/figures_and_data/normal_depth_image.png', depth_image_uint8)


            
            # 以下是另一种方法，与bridge.imgmsg_to_cv2函数等价，展开描述如何解析深度图像
            # 将每4个字节的二进制数据解析为float
            depth_values_ori = []
            for i in range(0, len(depth_msg.data), n):
                depth_value_bytes = bytes(depth_msg.data[i:i+n])
                # depth_value_float = struct.unpack('f', depth_value_bytes)[0]
                depth_value_float = struct.unpack('H', depth_value_bytes)[0]
                depth_values_ori.append(depth_value_float)
            # print("解析后的深度值:", depth_values_ori)
            
            # 将depth_values_ori按照depth_msg的行列数转换为深度图像
            depth_values = np.array(depth_values_ori)
            print(np.size(depth_values))

            self.depth_image = depth_values.reshape(depth_msg.height, depth_msg.width)
            # 存储转换后的深度图像为一个新的txt文件
            with open(self.depth_file_processed, 'w') as file:
                for i in range(depth_msg.height):
                    for j in range(depth_msg.width):
                        file.write(str(self.depth_image[i][j]) + ' ')
                    file.write('\n')
            # # 画出深度图像
            # # 关闭图形的边框
            # #plt.axis('off')
            # # 去除图片外面的部分
            # plt.axis('image')

            # # 确保紧凑的布局
            # plt.tight_layout()
            # plt.imshow(self.depth_image, cmap='gray')
            # plt.savefig('/image_process/figures_and_data/depth_image.png')
            
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 获取特定像素的深度值
        #depth_value = self.depth_image[self.row, self.col]
        #rospy.loginfo("Depth value at pixel ({}, {}): {}".format(self.row, self.col, depth_value))
        # This function is to calculate the real position in 3D world based on the pixel point
        

    def camera_base_link(self):
        #[fx, 0, cx] 
        #[0, fy, cy]
        #[0, 0, 1]
        #the intrinsic matrix of the camera
        #[347.99755859375, 0.0, 320.0] 
        #[0.0, 347.99755859375, 240.0]
        #[0.0, 0.0, 1.0]
        while(True):
            if(self.depth_image and self.extracted_image is not None):
                break
            
        cx = 320
        cy = 240
        z = self.depth_image[self.cX][self.cY]
        f = 347.99755859375
        x = (self.cX-cx)*z/f
        y = (self.cY-cy)*z/f

        listener = tf.TransformListener()

        source_frame = "base_link"  # 源坐标系
        target_frame = "d435_depth_optical_frame"  # 目标坐标系

        try:
            listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))
            (translation, rotation) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            print(rotation)
            # 构造齐次变换矩阵
            homogeneous_transform = tf.transformations.compose_matrix(translate=translation, angles=tf.transformations.euler_from_quaternion(rotation))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to obtain homogeneous transform between '{}' and '{}'".format(source_frame, target_frame))
        position = [[x],[y],[z],[1]]
        a = np.dot(homogeneous_transform,position)
        return a

    def main(self):
        #x,y,z = self.camera_base_link(self)

        
        rospy.spin()


if __name__ == "__main__":
    node = ImageProcess()
    node.main()
