#!/usr/bin/env python

import rospy
import argparse
import struct
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import rospy
import tf
import numpy as np



depth_value_bytes = bytes([1,1])
depth_value_float = struct.unpack('H', depth_value_bytes)[0]
print(depth_value_float)

rotation = [[0,0,1],
            [1,0,0],
            [0,1,0]]
angle = tf.transformations.euler_from_quaternion(rotation)
print("angle:",angle)