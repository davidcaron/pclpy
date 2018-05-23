import math
import os
import numpy as np
import pytest

import pclpy
from pclpy import pcl


def test_data(*args):
    return os.path.join("test_data", *args)


def make_pt(x, y, z):
    pt = pcl.point_types.PointXYZRGBA()
    pt.x = x
    pt.y = y
    pt.z = z
    return pt


def test_region_growing():
    pc = pclpy.io.read(test_data("bf.las"), "PointXYZRGBA")
    rg = pcl.segmentation.RegionGrowing.PointXYZRGBA_Normal()
    rg.setInputCloud(pc)
    normals_estimation = pcl.features.NormalEstimationOMP.PointXYZRGBA_Normal()
    normals_estimation.setInputCloud(pc)
    normals = pcl.PointCloud.Normal()
    normals_estimation.setRadiusSearch(0.1)
    normals_estimation.compute(normals)
    rg.setInputNormals(normals)

    clouds = []
    for n, th in enumerate([0.01, 10, 100, 1000]):
        rg.setMaxClusterSize(10000000)
        rg.setMinClusterSize(100)
        rg.setNumberOfNeighbours(15)
        rg.setSmoothnessThreshold(5 / 180 * math.pi)
        rg.setCurvatureThreshold(20)
        rg.setResidualThreshold(th)
        clusters = pcl.vectors.PointIndices()
        rg.extract(clusters)
        cloud = rg.getColoredCloud()
        clouds.append(cloud)
        pclpy.io.write(cloud, test_data("bf_rg%s.las" % n))

    # pclpy.view.vtk.view_multiple(*clouds)

