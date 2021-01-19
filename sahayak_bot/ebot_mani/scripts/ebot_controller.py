#! /usr/bin/env python

import rospy
import rospkg
import math

import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle

from pcl_filters import *
from pcl_helper import *
from features import *
from controller import Ur5Moveit

from object_msgs.msg import ObjectPose
from std_msgs.msg import Header
from ebot_mani.srv import PickPlace

class EBotUR5:
    def __init__(self):
        print("Initiated")
        self.color_list = []
        rospy.init_node('ebot_moveit_ur5')
        self.sub = rospy.Subscriber('/camera2/depth/points2', pc2.PointCloud2, self.pcl_callback, queue_size=1)
        self.pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
        self.pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)
        self.object_pose_pub  = rospy.Publisher("/detection_info", ObjectPose, queue_size=1)

    def pcl_callback(self, msg):
        pcl_cloud = ros_to_pcl(msg)
        # stats_outlier_filtered = StatisticalOutlierFilter(pcl_cloud)
        voxel = VoxelFilter(pcl_cloud)

        passthrough = PassthroughFilter(voxel, axis='z', min=0, max=1)
        passthrough = PassthroughFilter(passthrough, axis='x', min=-1, max=1)
        passthrough = PassthroughFilter(passthrough, axis='y', min=-1, max=1)

        objects, table = RansacPlaneSeg(passthrough)
        cluster_indices, white_cloud = EuclideanClustering(objects)
        cluster_cloud = ClusterMasking(cluster_indices, white_cloud, self.color_list)

        ros_cloud_objects = pcl_to_ros(objects, FRAME_ID)
        ros_cluster_cloud = pcl_to_ros(cluster_cloud, FRAME_ID)

        self.pcl_objects_pub.publish(ros_cloud_objects)
        self.pcl_cluster_pub.publish(ros_cluster_cloud)

        detected_objects_labels = []
        pose_list = dict()

        for index, pts_list in enumerate(cluster_indices):
            pcl_cluster = objects.extract(pts_list)
            ros_cluster = pcl_to_ros(pcl_cluster)

            c_hists = compute_color_histograms(ros_cluster, using_hsv=True)
            normals = get_normals(ros_cluster)
            n_hists = compute_normal_histograms(normals)

            feature = np.concatenate((c_hists, n_hists))

            prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
            label = encoder.inverse_transform(prediction)[0]
            detected_objects_labels.append(label)

            points_arr = ros_to_pcl(ros_cluster).to_array()
            object_centroid = np.mean(points_arr, axis=0)[:3]
            object_centroid[0] = np.asscalar(object_centroid[0])
            object_centroid[1] = np.asscalar(object_centroid[1])
            object_centroid[2] = np.asscalar(object_centroid[2])

            object_pose = ObjectPose()
            header = Header()
            header.seq = 1
            header.stamp = rospy.Time.now()
            header.frame_id = FRAME_ID

            object_pose.name = label
            object_pose.pose.header = header
            object_pose.pose.pose.position.x = object_centroid[0]
            object_pose.pose.pose.position.y = object_centroid[1]
            object_pose.pose.pose.position.z = object_centroid[2]
            self.object_pose_pub.publish(object_pose)
            pose_list[object_pose.name] = object_pose

        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
        order = ['coke_can', 'battery', 'glue']
        for o in order:
            if o in pose_list:
                obj_pose = pose_list[o]
            else:
                continue
            rospy.wait_for_service('pick_place')
            try:
                rospy.loginfo("Goint for {}".format(obj_pose.name))
                service = rospy.ServiceProxy('pick_place', PickPlace)
                if not service(obj_pose):
                    break
                    print('pick_place failed')
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        rospy.signal_shutdown('goal_reached')
        # if self.sub.get_num_connections() >= 1:
        #     self.sub.unregister()

if __name__ == '__main__':
    PKG_PATH = rospkg.RosPack().get_path('ebot_mani')
    FRAME_ID = 'camera_depth_frame2'

    model = pickle.load(open(PKG_PATH + '/scripts/svm/model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    ebot = EBotUR5()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except KeyboardInterrupt as e:
        print(e)
