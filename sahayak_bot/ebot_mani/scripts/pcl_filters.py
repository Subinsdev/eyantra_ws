#!/usr/bin/env python

import pcl
from pcl_helper import *

# STATISTICAL OUTLIER FILTER
def StatisticalOutlierFilter(cloud, kmean=3, x=0.00001):
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(kmean)
    outlier_filter.set_std_dev_mul_thresh(x)
    return outlier_filter.filter()

# VOXEL FILTER
def VoxelFilter(cloud, LEAF_SIZE=0.01):
    vox = cloud.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    return vox.filter()

# PASSTHROUGH FILTER
def PassthroughFilter(cloud, axis='z', min=0, max=1):
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name(axis)
    passthrough.set_filter_limits(min, max)
    return passthrough.filter()

# RANSAC PLANE SEGMENTATION
def RansacPlaneSeg(cloud, dist_thresh=0.01):
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(dist_thresh)
    inliers, coefficients = seg.segment()

    cloud_table = cloud.extract(inliers, negative=False)
    cloud_objects = cloud.extract(inliers, negative=True)
    return cloud_objects, cloud_table

# EUCLIDEAN CLUSTERING - DPSCAN
def EuclideanClustering(cloud, LEAF_SIZE=0.01, min_size=10, max_size=2500):
    white_cloud = XYZRGB_to_XYZ(cloud)
    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(LEAF_SIZE*2)
    ec.set_MinClusterSize(min_size)
    ec.set_MaxClusterSize(max_size)
    ec.set_SearchMethod(tree)
    return ec.Extract(), white_cloud

# CLUSTER MASKING
def ClusterMasking(cluster_indices, white_cloud, color_list):
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices), color_list)
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud

if __name__ == '__main__':
    cloud = pcl.load_XYZRGB('tabletop.pcd')

    pcl.save(downsampled, 'voxeled.pcd')
    pcl.save(filtered, 'passed.pcd')
    pcl.save(cloud_table, 'cloud_table.pcd')
    pcl.save(cloud_objects, 'cloud_objects.pcd')
    pcl.save(white_cloud, 'white_cloud.psd')
    pcl.save(cluster_indices, 'cluster_indices.pcd')
