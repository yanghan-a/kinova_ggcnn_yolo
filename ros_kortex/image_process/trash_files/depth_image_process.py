#!/usr/bin/env python

import rospy
import argparse
import struct
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt

#observing part of the data we find that 10 pixels is equivalent to 1cm
class DepthSubscriber:
    def __init__(self, row, col, output_file):
        rospy.init_node('depth_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/d435/depth/image_raw', Image, self.depth_callback)
        self.row = row
        self.col = col
        self.output_file = output_file


    def depth_callback(self, depth_msg):

        # my data is encoded as 16UC1 but not 32FC1, each pixel needs only 2 bytes to express
        n = 2

        try:
            with open(self.output_file, 'w') as file:
                file.write(str(depth_msg))
            print("Length of depth_msg data: ", len(depth_msg.data)) # 614400
            # 将深度图像转换为OpenCV格式
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            print("the size of depth_image",np.size(depth_image))
            
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

            depth_image = depth_values.reshape(depth_msg.height, depth_msg.width)
            # 存储转换后的深度图像为一个新的txt文件
            with open('depth_image.txt', 'w') as file:
                for i in range(depth_msg.height):
                    for j in range(depth_msg.width):
                        file.write(str(depth_image[i][j]) + ' ')
                    file.write('\n')
            # 画出深度图像
            plt.imshow(depth_image, cmap='gray')
            plt.savefig('depth_image.png')
            

        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 获取特定像素的深度值
        depth_value = depth_image[self.row, self.col]

        
        rospy.loginfo("Depth value at pixel ({}, {}): {}".format(self.row, self.col, depth_value))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth Subscriber with row and col parameters for Intel Realsense')
    parser.add_argument('--row', type=int, default=240, help='Row value for pixel (default: 240)')
    parser.add_argument('--col', type=int, default=320, help='Column value for pixel (default: 320)')
    parser.add_argument('--output_file', type=str, default='depth_msg.txt', help='Output file for depth value (default: depth_value.txt)')
    args = parser.parse_args()

    try:
        depth_subscriber = DepthSubscriber(args.row, args.col, args.output_file)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

