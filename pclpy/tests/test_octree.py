import pytest
import os
import numpy as np

from pclpy import pcl
import pclpy


def test_data(*args):
    return os.path.join("test_data", *args)


def test_voxel_centroid_simple():
    pc = pclpy.io.las.read(test_data("street_thinned.las"), "PointXYZ")
    vox = pcl.octree.OctreePointCloudVoxelCentroid.PointXYZ(0.3)
    vox.setInputCloud(pc)
    vox.addPointsFromInputCloud()
    centroids = pcl.PointCloud.PointXYZ()
    vox.getVoxelCentroids(centroids.points)
    assert centroids.size() == 3148

