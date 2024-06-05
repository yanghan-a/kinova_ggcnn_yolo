import rospy
import tf
import numpy as np

class Transform_process:
    def __init__(self):
        rospy.init_node("tf_listener")

        listener = tf.TransformListener()

        rate = rospy.Rate(1.0)
        source_frame = "base_link"  # 源坐标系
        target_frame = "camera_depth_optical_frame"  # 目标坐标系
        self.camera_world = None
        self.object_camera = None

        #获取物体到相机的矩阵
        # 文件路径
        file_path = '/home/yh/contact_graspnet/contact_graspnet/results/predictions_my_1.npz'
        # 加载 .npz 文件
        data = np.load(file_path, allow_pickle=True)

        print(data.files)
        # 获取文件中的数组
        pred_grasps_cam = data['pred_grasps_cam']
        pred_grasps_cam_dict = pred_grasps_cam.item()

        scores = data['scores']
        scores_dict = scores.item()

        contact_pts = data['contact_pts']
        contact_pts_dict = contact_pts.item()

        transform_matrixes = pred_grasps_cam_dict[-1]
        transform_scores = scores_dict[-1]
        transform_pts = contact_pts_dict[-1]


        # 找到最大得分的索引
        max_score_index = np.argmax(transform_scores)

        # print("最好的地方")
        # print(transform_matrixes[max_score_index])
        # print(transform_scores[max_score_index])
        # print(transform_pts[max_score_index])
        self.object_camera = transform_matrixes[max_score_index]
        # 关闭文件
        data.close()
        #获取相机到极坐标系的矩阵
        while not rospy.is_shutdown():
            try:
                # 等待变换可用
                listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))
                # 获取两个坐标系之间的变换
                (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
                # 将变换转换为变换矩阵
                translation = np.array(trans)
                rotation = np.array(rot)
                transform_matrix = tf.transformations.quaternion_matrix(rotation)
                transform_matrix[:3, 3] = translation
                self.camera_world = transform_matrix
                print("Transformation matrix from camera to world:")
                print(self.camera_world)
                self.object_camera = np.array([[ 0.94540906,  0.2900759 , -0.14851876,  0.18001969],
                                                [-0.20277731,  0.88038933,  0.42871466,  0.07036582],
                                                [ 0.2551141 , -0.3751945 ,  0.89114875,  0.8473753 ],
                                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
                print("Transformation matrix from object to camera:")
                print(self.object_camera)

                a = np.dot(self.camera_world,self.object_camera)
                print('最终矩阵')
                print(a)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get transform between target_frame and source_frame.")
            rate.sleep()
    
    def main(self):

        rospy.spin()


if __name__ == "__main__":
    node = Transform_process()
    node.main()



