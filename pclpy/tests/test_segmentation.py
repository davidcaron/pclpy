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


# def test_min_cut_segmentation():
#     pc = pclpy.io.read_las(test_data("bf.las"))
#     seg = pcl.segmentation.RegionGrowing.PointXYZRGBA()
#     print(dir(seg))
#     seg.setInputCloud(pc)
#
#     foreground = pcl.PointCloud.PointXYZRGBA()
#     foreground.push_back(make_pt(5.06, 8.03, 22.16))
#     seg.setForegroundPoints(foreground)
#
#     # background = pcl.PointCloud.PointXYZRGBA()
#     # background.push_back(make_pt(4.848, 8.426, 21.636))
#     # background.push_back(make_pt(5.413, 7.513, 21.710))
#     # seg.setBackgroundPoints(background)
#
#     clouds = []
#     for n, neighbors in enumerate([10, 15, 25]):
#         seg.setSigma(0.25)
#         seg.setRadius(1.5)
#         seg.setNumberOfNeighbours(neighbors)
#         seg.setSourceWeight(0.8)
#
#         clusters = pcl.vector.PointIndices()
#         seg.extract(clusters)
#         seg.getColoredCloud()
#         cloud = seg.getColoredCloud()
#         clouds.append(cloud)
#         pclpy.io.to_las(cloud, test_data("bf_min_cut%s.las" % n))
#
#     pclpy.view.vtk.view_multiple(clouds)

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

