import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def color_extraction_callback(image_msg):
    # 将ROS图像消息转换为OpenCV图像
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

    # 转换图像颜色空间为HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    print(np.size(hsv_image))#921600 = 480*640*3
    # 定义颜色范围（例：提取红色）
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # 创建掩膜（只保留颜色范围内的像素）
    mask = cv2.inRange(hsv_image, lower_red, upper_red)

    # 应用掩膜到原始图像
    extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    print("extracted_image:",np.size(extracted_image))
    # 寻找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 寻找最大轮廓
    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)

        # 计算轮廓的中心坐标
        M = cv2.moments(max_contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # 在图像上绘制中心点
        cv2.circle(extracted_image, (cX, cY), 5, (0, 255, 0), -1)
        print("cX:",cX)
        print("cY:",cY)
    # 保存提取的图像到文件
    cv2.imwrite("color_image.jpg", extracted_image)

if __name__ == "__main__":
    rospy.init_node("color_extraction_example")

    # 订阅彩色图像话题
    color_image_topic = "d435/color/image_raw"
    rospy.Subscriber(color_image_topic, Image, color_extraction_callback)

    rospy.spin()