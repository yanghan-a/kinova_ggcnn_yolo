#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
import threading
import yaml

from sensor_msgs.msg import Image, CompressedImage
from detection_msgs.msg import BoundingBox, BoundingBoxes

np.seterr(divide='ignore', invalid='ignore')
torch.backends.cudnn.enabled = False
# add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox

class KalmanFilter:
    def __init__(self, state_dim, measure_dim):
        self.kalman = cv2.KalmanFilter(state_dim, measure_dim)
        self.kalman.measurementMatrix = np.eye(measure_dim, state_dim, dtype=np.float32)
        self.kalman.transitionMatrix = np.eye(state_dim, dtype=np.float32)
        self.kalman.processNoiseCov = np.eye(state_dim, dtype=np.float32) * 1e-4
        self.kalman.measurementNoiseCov = np.eye(measure_dim, dtype=np.float32) * 1e-1
        self.kalman.errorCovPost = np.eye(state_dim, dtype=np.float32)
        self.kalman.statePost = np.zeros((state_dim, 1), dtype=np.float32)

    def predict(self):
        return self.kalman.predict()

    def correct(self, measurement):
        return self.kalman.correct(measurement)

class EMA:
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.ema = None

    def update(self, value):
        if self.ema is None:
            self.ema = value
        else:
            self.ema = self.alpha * value + (1 - self.alpha) * self.ema
        return self.ema


@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        # self.conf_thres = rospy.get_param("~confidence_threshold")
        self.conf_thres = 0.5
        # self.iou_thres = rospy.get_param("~iou_threshold")
        self.iou_thres = 0.45
        # self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.agnostic_nms = "true"
        # self.max_det = rospy.get_param("~maximum_detections")
        self.max_det = 1000
        # self.classes = rospy.get_param("~classes", 63) #只显示指定的类别 
        self.classes = None
        # self.line_thickness = rospy.get_param("~line_thickness")
        self.line_thickness = 3
        # self.view_image = rospy.get_param("~view_image")
        self.view_image = False
        # Initialize weights 
        # weights = rospy.get_param("~weights")
        weights = "yolov5m.pt"
        # Initialize model
        # self.device = select_device(str(rospy.get_param("~device","")))
        self.device = select_device("0")
        # self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=True, data="yolov5/data/coco128.yaml")
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        # self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = [640, 640]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        # self.half = rospy.get_param("~half", False)
        self.half = False
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup        
        
        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type("/camera/color/color_crop_image", blocking = True)
        # self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"
        self.compressed_input = None # do not use compressed image

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            "/yolov5/detections", BoundingBoxes, queue_size=10
        )
        # Initialize image publisher
        # self.publish_image = rospy.get_param("~publish_image")
        self.publish_image = True
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                "/yolov5/image_out", Image, queue_size=10
            )
            self.image_pub_masked = rospy.Publisher(
                "/yolov5/image_out_masked", Image, queue_size=10
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

        self.kalman_filters = {}
        self.ema_filters = {}

                # Read YAML file
        yaml_path = "yolov5/data/coco128.yaml"
        with open(yaml_path, 'r') as file:
            yaml_content = yaml.safe_load(file)
        
        self.name_to_id = {v: k for k, v in yaml_content['names'].items()}
    
    def init_kalman_filter(self, bbox_id):
        # Initialize a Kalman filter for each bounding box
        self.kalman_filters[bbox_id] = KalmanFilter(state_dim=4, measure_dim=2)

    def init_ema_filter(self, bbox_id):
        # Initialize an EMA filter for each bounding box
        self.ema_filters[bbox_id] = EMA(alpha=0.5)

    def callback(self, data):
        """adapted from yolov5/detect.py"""
        # print(data.header)
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        im, im0 = self.preprocess(im)
        im0_masked = im0.copy()
        # print(im.shape)
        # print(img0.shape)
        # print(img.shape)

        # Run inference
        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        ### To-do move pred to CPU and fill BoundingBox messages
        
        # Process predictions 
        det = pred[0].cpu().numpy()

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header
        
        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        annotator_masked = Annotator(im0, line_width=self.line_thickness, example=str(self.names))

        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for i, (*xyxy, conf, cls) in enumerate(reversed(det)):
                bounding_box = BoundingBox()
                c = int(cls)
                bbox_id = f"{c}_{i}"  # Unique ID for each bbox based on class and index

                if bbox_id not in self.kalman_filters:
                    self.init_kalman_filter(bbox_id)

                if bbox_id not in self.ema_filters:
                    self.init_ema_filter(bbox_id)


                # Get bbox center and size for Kalman filter
                cx, cy = (xyxy[0] + xyxy[2]) / 2, (xyxy[1] + xyxy[3]) / 2
                w, h = xyxy[2] - xyxy[0], xyxy[3] - xyxy[1]
                measurement = np.array([cx, cy], dtype=np.float32).reshape(2, 1)

                # Apply Kalman filter
                self.kalman_filters[bbox_id].correct(measurement)
                predicted_bbox = self.kalman_filters[bbox_id].predict()

                # Update bounding box with smoothed values
                smoothed_cx, smoothed_cy = predicted_bbox[0], predicted_bbox[1]
                smoothed_bbox = [
                    smoothed_cx - w / 2,
                    smoothed_cy - h / 2,
                    smoothed_cx + w / 2,
                    smoothed_cy + h / 2
                ]

                smoothed_conf = self.ema_filters[bbox_id].update(conf)
                
                # print(f"smotthed_bboxL {smoothed_bbox}")
                # Fill in bounding box message
                bounding_box.Class = self.names[c]
                bounding_box.probability = smoothed_conf
                bounding_box.xmin = int(smoothed_bbox[0][0])
                bounding_box.ymin = int(smoothed_bbox[1][0])
                bounding_box.xmax = int(smoothed_bbox[2][0])
                bounding_box.ymax = int(smoothed_bbox[3][0])
                bounding_boxes.bounding_boxes.append(bounding_box)

                # print(f"xyxy: {xyxy}")

                bounding_boxes.bounding_boxes.append(bounding_box)

                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                      # integer class
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))   
                    annotator_masked.ROI_mask(xyxy)    

                
                ### POPULATE THE DETECTION MESSAGE HERE

            # Stream results
            im0 = annotator.result(return_mask=False)
            im0_masked = annotator_masked.result(return_mask=True)
            im0_masked = cv2.resize(im0_masked,(300,300),cv2.INTER_NEAREST)

        # Publish prediction
        self.pred_pub.publish(bounding_boxes)

        # Publish & visualize images
        if self.view_image:
            cv2.imshow(str(0), im0)
            # print(f"im0_masked shape: {im0.shape}")
            cv2.waitKey(1)  # 1 millisecond
        if self.publish_image and len(det):
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
            self.image_pub_masked.publish(self.bridge.cv2_to_imgmsg(im0_masked, "passthrough"))
        

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 

def input_listener(detector):
    while not rospy.is_shutdown():
        try:
            user_input = input("Enter new class ID (or 'exit' to quit): ")
            if user_input.lower() == 'exit':
                rospy.signal_shutdown("User requested shutdown")
                break
            if user_input.lower() == 'all':
                detector.classes = None
            else:
                class_id = detector.name_to_id[user_input]
                # class_id = int(user_input)
                detector.classes = [class_id]
            print(f"Updated classes to: {detector.classes}")
        except ValueError:
            print("Invalid input. Please enter a valid class ID.")


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()

    input_thread = threading.Thread(target=input_listener, args=(detector,))
    input_thread.daemon = True
    input_thread.start()

    rospy.spin()
