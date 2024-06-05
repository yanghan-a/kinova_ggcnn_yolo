import numpy as np
import tf.transformations as transformations

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles (roll, pitch, yaw) to a quaternion.
    The Euler angles are assumed to be in radians.
    """
    quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

# 示例用法
roll = -1.29027  # 角度
pitch = -1.68564  # 弧度
yaw =  94.492  # 弧度


roll_rad = np.deg2rad(roll)
pitch_rad = np.deg2rad(pitch)
yaw_rad = np.deg2rad(yaw)
# 转换为四元数
print(roll_rad,pitch_rad,yaw_rad)
quaternion = euler_to_quaternion(roll, pitch, yaw)
print("Quaternion: ", quaternion)

#Quaternion:  [ 0.32445017  0.64077692  0.38078925 -0.58235434]
