from os import path
import sys

import cv2
import numpy as np
import scipy.ndimage as ndimage

# import tensorflow as tf
import tensorflow.compat.v1 as tf
from tensorflow.keras.models import load_model
from tensorflow.python.keras import backend as K

#from tensorflow.keras.backend import set_session

sys.path.append("/home/yh/ggcnn_mvp/src/mvp_grasp/dougsm_helpers/src")
sys.path.append("/home/yh/ggcnn_mvp/src/mvp_grasp/ggcnn/src/ggcnn")

from dougsm_helpers.timeit import TimeIt

MODEL_FILE = 'models/epoch_29_model.hdf5'
# sess = tf.Session()
# sess = tf.Session()

tf.compat.v1.disable_eager_execution() # 这就添加至上句代码之后

config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = 0.65 #占用85%显存
sess = tf.Session(config=config)




K.set_session(sess)
graph = tf.get_default_graph()
model = load_model(path.join(path.dirname(__file__), MODEL_FILE))


TimeIt.print_output = False  # For debugging/timing

#整体来说，这个函数就是对特殊值nan进行处理，并把图像修建放缩到300*300
def process_depth_image(depth, crop_size, out_size=300, return_mask=False, crop_y_offset=0,crop_x_offset=0):
    imh, imw = depth.shape

    with TimeIt('Process Depth Image'):
        with TimeIt('Crop'):
            # Crop.
            depth_crop = depth[(imh - crop_size) // 2 - crop_y_offset:(imh - crop_size) // 2 + crop_size - crop_y_offset,
                               (imw - crop_size) // 2- crop_x_offset:(imw - crop_size) // 2 + crop_size- crop_x_offset]

        # Inpaint
        # OpenCV inpainting does weird things at the border.
        with TimeIt('Inpainting_Processing'):
            depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
            depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)
            # print("depth_nan",depth_nan_mask[0][0])
            kernel = np.ones((3, 3),np.uint8)
            depth_nan_mask = cv2.dilate(depth_nan_mask, kernel, iterations=1)

            depth_crop[depth_nan_mask==1] = 0

            # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
            depth_scale = np.abs(depth_crop).max()
            depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.

            with TimeIt('Inpainting'):
                #这里对于nan的值的处理是选取周围的像素，调用cv2.INPAINT_NS来求解这点的像素
                depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)

            # Back to original size and value range.
            depth_crop = depth_crop[1:-1, 1:-1]
            depth_crop = depth_crop * depth_scale

        with TimeIt('Resizing'):
            # Resize
            #cv2.INTER_ARE这个算法通常用与大调小
            # 缩放深度值范围到0到255之间
            # depth_file = '/home/yh/ggcnn_mvp/src/mvp_grasp/depth_data.txt'

            # with open(depth_file, 'w') as file:
            #     for i in range(np.size(depth_crop,0)):
            #         for j in range(np.size(depth_crop,1)):
            #             file.write(str(depth_crop[i][j]) + ' ')
            #         file.write('\n')
            # depth_image_normalized = cv2.normalize(depth_crop, None, 0, 255, cv2.NORM_MINMAX)

            # 将深度图像的数据类型转换为uint8
            # depth_crop = cv2.convertScaleAbs(depth_image_normalized)
            # cv2.imwrite("/home/yh/ggcnn_mvp/src/mvp_grasp/depth_image4.png", depth_crop)
            depth_crop = cv2.resize(depth_crop, (out_size, out_size), cv2.INTER_AREA)
            # depth_image_normalized = cv2.normalize(depth_crop, None, 0, 255, cv2.NORM_MINMAX)
            
            # # 将深度图像的数据类型转换为uint8
            # depth_crop = cv2.convertScaleAbs(depth_image_normalized)
            # cv2.imwrite("/home/yh/ggcnn_mvp/src/mvp_grasp/depth_image2.png", depth_crop)
        if return_mask:
            with TimeIt('Return Mask'):
                depth_nan_mask = depth_nan_mask[1:-1, 1:-1]
                depth_nan_mask = cv2.resize(depth_nan_mask, (out_size, out_size), cv2.INTER_NEAREST)
            return depth_crop, depth_nan_mask
        else:
            return depth_crop


def predict(depth, process_depth=True, crop_size=300, out_size=300, depth_nan_mask=None, filters=(2.0, 1.0, 1.0), crop_y_offset=0,crop_x_offset=0):
    global graph, sess
    if process_depth:
        depth, depth_nan_mask = process_depth_image(depth, crop_size, out_size, True, crop_y_offset=crop_y_offset,crop_x_offset=crop_x_offset)

    # Inference
    depth = np.clip((depth - depth.mean()), -1, 1)
    K.set_session(sess)
    with graph.as_default():
        pred_out = model.predict(depth.reshape((1, 300, 300, 1)))

    points_out = pred_out[0].squeeze()
    points_out[depth_nan_mask] = 0

    # Calculate the angle map.
    cos_out = pred_out[1].squeeze()
    sin_out = pred_out[2].squeeze()
    ang_out = np.arctan2(sin_out, cos_out) / 2.0

    width_out = pred_out[3].squeeze() * 150.0  # Scaled 0-150:0-1

    # Filter the outputs.
    if filters[0]:
        points_out = ndimage.filters.gaussian_filter(points_out, filters[0])  # 3.0
    if filters[1]:
        ang_out = ndimage.filters.gaussian_filter(ang_out, filters[1])
    if filters[2]:
        width_out = ndimage.filters.gaussian_filter(width_out, filters[2])

    points_out = np.clip(points_out, 0.0, 1.0-1e-3)

    return points_out, ang_out, width_out, depth
