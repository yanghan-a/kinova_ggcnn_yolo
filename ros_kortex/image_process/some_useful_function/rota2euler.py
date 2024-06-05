import numpy as np

def rotation_matrix_to_euler_xyz_fixed(R):
    # Extract rotation angles from rotation matrix
    if R[2, 0] < 1:
        if R[2, 0] > -1:
            y = np.arcsin(-R[2, 0])
            x = np.arctan2(R[2, 1], R[2, 2])
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            # Not a unique solution: x - z = atan2(-r10,r11)
            y = -np.pi / 2
            x = -np.arctan2(R[1, 2], R[1, 1])
            z = 0
    else:
        # Not a unique solution: x + z = atan2(-r10,r11)
        y = np.pi / 2
        x = np.arctan2(R[1, 2], R[1, 1])
        z = 0
        
    return np.array([x, y, z])

# Example usage:
# Define your rotation matrix
# R_goal = np.array([[     -1,        0,       0],
#                    [      0,   0.7071, -0.7071],
#                    [      0,  -0.7071, -0.7071]])
# R_current = np.array([[ 0,  0, 1],
#                       [-1,  0, 0],
#                       [ 0, -1, 0]])
# R = np.dot(R_goal,np.linalg.inv(R_current))

camera_bottom_screw_frame2end_effector_link = np.array([[-0.02014814,  0.07828623,  0.9967273 ,  0.04820097],
                                                        [-0.03098307 ,-0.99649975,  0.07764206, -0.01726086],
                                                        [ 0.99931682, -0.02931733  ,0.02250317 , 0.02901451],
                                                        [ 0.       ,   0.      ,    0.      ,    1.        ]])


tool_frame2base_link = np.array([[ 0.04371656, -0.99804331 ,-0.04470353, -0.04225469],
                                 [-0.99898416 ,-0.04318054 ,-0.01288719 , 0.36617767],
                                 [ 0.01093165 , 0.0452215 , -0.99891717 , 0.20315396],
                                 [ 0.     ,     0.    ,      0.    ,     1.        ]])

tool_frame2base_link_real = np.array([[-9.18086672e-03 ,-9.99900315e-01  ,1.07271593e-02 ,-7.09835067e-04],
            [-9.99947000e-01,  9.13025349e-03, -4.75771679e-03 , 2.94882548e-01],
            [ 4.65930083e-03 ,-1.07702707e-02, -9.99931144e-01 , 1.98944127e-01],
            [ 0.00000000e+00 , 0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]])


            # [[-0.69678274 -0.71693102  0.0224438   0.29488001]
            # [-0.71722784  0.69677409 -0.00949154 -0.33353833]
            # [-0.00883348 -0.02271086 -0.99970305  0.04160237]
            # [ 0.          0.          0.          1.        ]]
place_postion = np.array([[-0.69678274 ,-0.71693102 , 0.0224438 ,  0.29488001],
                          [-0.71722784 , 0.69677409 ,-0.00949154 ,-0.33353833],
                          [-0.00883348, -0.02271086 ,-0.99970305 , 0.04160237],
                          [ 0.      ,    0.        ,  0.          ,1.         ]])

R1 = camera_bottom_screw_frame2end_effector_link[:3, :3]

R2 = place_postion[:3, :3]
# Convert rotation matrix to XYZ Euler angles
euler_angles = rotation_matrix_to_euler_xyz_fixed(R2)
print("XYZ Euler Angles (radians):", euler_angles)

# Convert radians to degrees for better readability
euler_angles_degrees = np.degrees(euler_angles)
print("XYZ Euler Angles (degrees):", euler_angles_degrees)


# 这个是相机标定后计算camera_bottom_screw_frame2end_effector_link位置的欧拉角表示
# XYZ Euler Angles (radians): [-1.59100787  0.07836642 -3.11218083]
# XYZ Euler Angles (degrees): [ -91.15803609    4.49006496 -178.31482659]
