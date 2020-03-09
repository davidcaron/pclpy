import pytest
import os
import numpy as np

from pclpy import pcl
import pclpy
from .utils import test_data


def test_voxel_centroid_simple():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    vox = pcl.octree.OctreePointCloudVoxelCentroid.PointXYZ(0.3)
    vox.setInputCloud(pc)
    vox.addPointsFromInputCloud()
    centroids = pcl.PointCloud.PointXYZ()
    vox.getVoxelCentroids(centroids.points)
    assert centroids.size() == 3148


def test_voxel_centroid_api():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output = pclpy.octree_voxel_downsample(pc, resolution=0.3, centroids=True)
    assert output.size() == 3148


def test_voxel_centroid_api_rgba():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZRGBA")
    output = pclpy.octree_voxel_downsample(pc, resolution=0.3, centroids=False)
    assert output.size() == 3148
