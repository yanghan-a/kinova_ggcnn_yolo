import numpy as np

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Converts Euler angles (roll, pitch, yaw) to a rotation matrix.
    The Euler angles are assumed to be in radians.
    """
    # Compute individual rotation matrices
    R_x = np.array([[1.0, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1.0, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1.0]])
    
    # Combined rotation matrix
    temp = np.dot(R_y, R_x)
    R = np.dot(R_z, temp)
    
    return R

def euler_to_homogeneous_transform(roll, pitch, yaw, tx, ty, tz):
    """
    Converts Euler angles and translation to a homogeneous transformation matrix.
    The Euler angles are assumed to be in radians.
    """
    # Get the rotation matrix from Euler angles
    R = euler_to_rotation_matrix(roll, pitch, yaw)
    
    # Create the homogeneous transformation matrix
    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    
    return T

# Example usage
roll = np.deg2rad(-1.29)   # Roll angle in radians
pitch = np.deg2rad(-1.68)  # Pitch angle in radians
yaw = np.deg2rad(94.492)   # Yaw angle in radians
tx, ty, tz = 0.0618165, -0.0340575, 0.0393755  # Translation components

T = euler_to_homogeneous_transform(roll, pitch, yaw, tx, ty, tz)
print("Homogeneous Transformation Matrix:\n", T)
