import numpy as np
import math
def rotation_matrix_to_euler_angles(R):
    """
    Convert rotation matrix to Euler angles (yaw, pitch, roll).
    Assumes the rotation matrix represents rotations in ZYX order.
    """
    # Extract rotation angles
    theta_x = np.arctan2(R[2, 1], R[2, 2])
    theta_y = np.arctan2(-R[2, 0], math.sqrt(R[2,1]**2+R[2,2]**2))
    theta_z = np.arctan2(R[1, 0], R[0, 0])

    return np.array([theta_x, theta_y, theta_z])

# Example transformation matrix
# T = np.array([[0.5068978, 0.8620037, 0.00203395, 0.34276867],
#               [-0.8141765, 0.47799549, 0.32960108, 0.00825789],
#               [0.28314504, -0.16873007, 0.9441181, 0.45193815],
#               [0.0, 0.0, 0.0, 1.0]])
T = np.array([[ 0.20277731, -0.88038933, -0.42871466,  0.34713418],
                [-0.94561191, -0.28977703,  0.14780907,  0.23260178],
                [-0.25436116, 0.37542538, -0.89126674,  0.14184948],
                [ 0.0,          0.0,        0.0  ,        1.0       ]])

# 最终矩阵
# [[ 0.80370724  0.59010905 -0.07632801  0.53359223]
#  [ 0.59497405 -0.7953226   0.11605118  0.49515144]
#  [ 0.00777747 -0.13868435 -0.99030612  0.14061684]
#  [ 0.          0.          0.          1.        ]]

# Extract rotation matrix
R = T[:3, :3]

# Convert rotation matrix to Euler angles
euler_angles = rotation_matrix_to_euler_angles(R)
print("theta_x, theta_y, theta_z (degrees):", np.degrees(euler_angles))
