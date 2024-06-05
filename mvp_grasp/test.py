import cv2

# 读取图像
image = cv2.imread("color_image copy 2.png")
imh = 480
imw = 640
crop_size = 300
crop_offset = 0
out_size = 300
crop_y_offset=0
crop_x_offset=0

# 定义裁剪区域 (y1:y2, x1:x2)
cropped_image = image[(imh - crop_size) // 2 - crop_y_offset:(imh - crop_size) // 2 + crop_size - crop_y_offset,
                               (imw - crop_size) // 2- crop_x_offset:(imw - crop_size) // 2 + crop_size- crop_x_offset]

# # 显示裁剪后的图像
# cv2.imshow("Cropped Image", cropped_image)
# cv2.waitKey(0)

# 保存裁剪后的图像
cv2.imwrite("cropped_image1.jpg", cropped_image)
