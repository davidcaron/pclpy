import pytest
import os
import numpy as np
import laspy

from pclpy import pcl
import pclpy


def test_data(*args):
    return os.path.join("test_data", *args)


def test_compute_normals_simple():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output = pclpy.compute_normals(pc, radius=1)
    assert np.any(output.normals)


def test_compute_normals_simple_given_output():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    pc.compute_normals(radius=1)
    pc2 = pcl.PointCloud.PointNormal(pc.xyz)
    assert np.allclose(pc.xyz, pc2.xyz)
    output = pclpy.compute_normals(pc2, radius=1, output_cloud=pc2)
    normals = output.normals
    nan = np.sum(np.isnan(normals), axis=1)
    assert np.sum(nan) != normals.shape[0]
    assert np.sum(nan) == 6
    norm = np.linalg.norm(normals[~nan], axis=1)
    assert np.allclose(norm, 1., atol=0.001)


def test_difference_of_normals_estimation_simple():
    pc = pclpy.read(test_data("street.las"), "PointXYZRGBA")

    pc_subsampled = pclpy.octree_voxel_downsample(pc, 0.05)
    subsampled = pclpy.octree_voxel_downsample(pc_subsampled, 0.4)

    normals_large = pcl.PointCloud.PointNormal()
    pc_subsampled.compute_normals(radius=4, output_cloud=normals_large, num_threads=8, search_surface=subsampled)
    normals_small = pcl.PointCloud.PointNormal()
    pc_subsampled.compute_normals(radius=0.10, output_cloud=normals_small, num_threads=8)

    don = pcl.features.DifferenceOfNormalsEstimation.PointXYZRGBA_PointNormal_PointNormal()
    don.setInputCloud(pc_subsampled)
    don.setNormalScaleLarge(normals_large)
    don.setNormalScaleSmall(normals_small)
    output = pcl.PointCloud.PointNormal(pc_subsampled.xyz)
    don.computeFeature(output)
    indices = np.argwhere(output.curvature > 0.4)
    output2 = pcl.PointCloud.PointNormal(output, pcl.vectors.Int(indices))

    clusters = pclpy.extract_clusters(pcl.PointCloud.PointXYZ(output2.xyz), 0.1, 50, 100000)
    assert len(clusters) == 32

    # display
    # indices = pcl.vectors.Int()
    # for cluster in clusters:
    #     indices.extend(cluster.indices)
    # output2 = pcl.PointCloud.PointNormal(output2, indices)
    # pclpy.show(pc, output2, point_xyz_random_color=True, overlay=False)
