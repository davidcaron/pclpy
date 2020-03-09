import math
import os
import numpy as np
import pytest

import pclpy
from pclpy import pcl
from .utils import test_data


def make_pt(x, y, z):
    pt = pcl.point_types.PointXYZRGBA()
    pt.x = x
    pt.y = y
    pt.z = z
    return pt


def test_supervoxel_clustering():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZRGBA")
    normals = pc.compute_normals(radius=0.3)
    vox = pcl.segmentation.SupervoxelClustering.PointXYZRGBA(voxel_resolution=1,
                                                             seed_resolution=2)
    vox.setInputCloud(pc)
    vox.setNormalCloud(normals)
    # vox.setSpatialImportance(2)
    # vox.setNormalImportance(2)
    # vox.setColorImportance(2)
    output = pcl.vectors.map_uint32t_PointXYZRGBA()
    vox.extract(output)
    assert len(list(output.items())) == 95


def test_region_growing():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZRGBA")
    rg = pcl.segmentation.RegionGrowing.PointXYZRGBA_Normal()
    rg.setInputCloud(pc)
    normals_estimation = pcl.features.NormalEstimationOMP.PointXYZRGBA_Normal()
    normals_estimation.setInputCloud(pc)
    normals = pcl.PointCloud.Normal()
    normals_estimation.setRadiusSearch(0.35)
    normals_estimation.compute(normals)
    rg.setInputNormals(normals)

    rg.setMaxClusterSize(1000000)
    rg.setMinClusterSize(10)
    rg.setNumberOfNeighbours(15)
    rg.setSmoothnessThreshold(5 / 180 * math.pi)
    rg.setCurvatureThreshold(5)
    rg.setResidualThreshold(1)
    clusters = pcl.vectors.PointIndices()
    rg.extract(clusters)
    assert max([len(c.indices) for c in clusters]) == 2449  # ground


def test_region_growing_simple():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZRGBA")
    normals = pc.compute_normals(radius=0.35)

    clusters = pclpy.region_growing(pc,
                                    normals=normals,
                                    min_size=10,
                                    max_size=1000000,
                                    n_neighbours=15,
                                    smooth_threshold=5,
                                    curvature_threshold=5,
                                    residual_threshold=1,
                                    )
    assert max([len(c.indices) for c in clusters]) == 2449  # ground


def test_euclidean_cluster():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZRGBA")  # type: pcl.PointCloud.PointXYZRGBA
    indices = pc.extract_clusters(tolerance=0.5, min_size=10, max_size=100000)
    assert len(indices[0].indices) == 4799
