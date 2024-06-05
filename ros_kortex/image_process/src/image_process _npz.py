import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
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

        # 订阅相机信息话题
        self.color_depth_info_topic = "/camera/depth/camera_info"
        self.camera_depth_info_sub = rospy.Subscriber(self.color_depth_info_topic, CameraInfo, self.camera_info_callback)
        # 订阅彩色图像话题
        self.color_image_topic = "/camera/color/image_raw"
        self.color_sub = rospy.Subscriber(self.color_image_topic, Image, self.color_callback)
        # subscribe depth image
        self.depth_image_topic = "/camera/depth/image_raw"
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)

        self.configuration_pub = rospy.Publisher('/my_pose', Pose, queue_size=10)
        self.output_file = 'depth_msg.txt'
        self.camera_info = None
        self.color_image = None
        self.depth_image = None

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def color_callback(self, image_msg):
        # 将ROS图像消息转换为OpenCV图像
        self.color_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

    def depth_callback(self, depth_msg):
        # 将ROS图像消息转换为OpenCV图像
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

    def save_data(self, filename):
        while not any([self.camera_info, self.color_image, self.depth_image]):
            rospy.sleep(0.1)  # 等待数据到达

        K = np.array(self.camera_info.K).reshape(3, 3)

        depth_image_m = self.depth_image.astype(np.float32) / 1000.0  # 深度转换为米

        np.savez(filename, K=K, color_image=self.color_image, depth_image=depth_image_m)
   

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
        target_frame = "camera_depth_optical_frame"  # 目标坐标系

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
        self.save_data('my_1.npz')
        self.save_data('home/yh/contact_graspnet/contact_graspnet/results/my_1.npz')
        
        rospy.spin()


if __name__ == "__main__":
    node = ImageProcess()
    node.main()
