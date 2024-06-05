#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf.transformations as transformations

def transform_to_matrix(transform):
    """
    Converts a TransformStamped message to a 4x4 transformation matrix.
    """
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    matrix = np.identity(4)
    
    # Set translation part
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z

    # Convert quaternion to rotation matrix
    q = [rotation.x, rotation.y, rotation.z, rotation.w]
    rot_matrix = transformations.quaternion_matrix(q)
    
    # Set rotation part
    matrix[:3, :3] = rot_matrix[:3, :3]

    return matrix

def main():
    rospy.init_node('tf_listener', anonymous=True)
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Here we specify the source and target frames
            source_frame = 'base_link'
            # target_frame = 'camera_aligned_depth_to_color_frame'
            target_frame = 'tool_frame'

            # target_frame = "camera_aligned_depth_to_color_frame"  # 目标坐标系
            
            # Get the transform between the two frames
            transform = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
            
            # Convert to transformation matrix
            transform_matrix = transform_to_matrix(transform)
            print(transform)
            # depth_optical2end_effector_link =  np.array( [[-0.07828623, -0.9967273 , -0.02014814,  0.0618165 ],
            #                                               [ 0.99649975, -0.07764206, -0.03098307, -0.0340575 ],
            #                                               [ 0.02931733, -0.02250317,  0.99931682,  0.0393755 ],
            #                                               [ 0.0       ,  0.0       ,    0.0     ,    1.0        ]])
            # camera_bottom_screw_frame2end_effector_link = np.dot(depth_optical2end_effector_link,transform_matrix)
            
            # 真实条件下的最佳初始位置
            # [[-9.18086672e-03 -9.99900315e-01  1.07271593e-02 -7.09835067e-04]
            # [-9.99947000e-01  9.13025349e-03 -4.75771679e-03  2.94882548e-01]
            # [ 4.65930083e-03 -1.07702707e-02 -9.99931144e-01  1.98944127e-01]
            # [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
            rospy.loginfo("Transformation Matrix from {} to {}: \n{}".format(target_frame, source_frame, transform_matrix))

            # rospy.loginfo("Transformation Matrix from {} to {}: \n{}".format('camera_bottom_screw_frame', 'end_effector_link', camera_bottom_screw_frame2end_effector_link))
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to get transform: {}".format(e))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
