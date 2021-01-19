#!/usr/bin/env python
import numpy as np
import pickle
import rospy
import rospkg

from ebot_mani.srv import GetNormals
from training_helper import *
from pcl_helper import *
from features import *
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# def get_normals(ros_cloud):
#     pcl_cloud = XYZRGB_to_XYZ(ros_to_pcl(ros_cloud))
#
#     ne = pcl_cloud.make_NormalEstimation()
#     tree = pcl_cloud.make_kdtree()
#     ne.set_SearchMethod(tree)
#     ne.set_RadiusSearch(0.03)
#     print(ne)
#     cloud_normals = ne.compute()
#
#     return pcl_to_ros(cloud_normals)


if __name__ == '__main__':
    rospy.init_node('prepare_dataset')
    models = [('coke_can', 'coke_can'),
              ('battery', 'soap2'),
              ('glue', 'glue')]

    # training_helper.initial_setup() --> disable gravity and delete ground plane
    initial_setup()
    labeled_features = []

    for model_name, model_path in models:
        # training_helper.spawn_model(model_name, pkg_name) --> spawns sdf model
        spawn_model(model_path, 'ebot_gazebo')

        for i in range(17):
            good_sample = False
            try_count = 0
            while not good_sample and try_count < 5:
                sample_cloud = capture_sample('/camera2/depth/points2')
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                if sample_cloud_arr.shape[0] == 0:
                    print('invalid cloud')
                    try_count += 1
                else:
                    good_sample = True

            c_hists = compute_color_histograms(sample_cloud, using_hsv=True)
            print(type(sample_cloud))
            normals = get_normals(sample_cloud)
            print(type(normals))
            n_hists = compute_normal_histograms(normals)

            feature = np.concatenate((c_hists, n_hists))
            labeled_features.append([feature, model_name])

        # training_helper.delete_model() --> deletes previously spawned model
        delete_model()

    pickle.dump(labeled_features, open(rospkg.RosPack().get_path('ebot_mani')+'/scripts/svm/training_set.sav', 'wb'))
