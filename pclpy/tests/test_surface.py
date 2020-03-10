import pytest
import os
import numpy as np
import laspy

from pclpy import pcl
import pclpy
from .utils import test_data


def test_moving_least_squares_normals():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output = pclpy.moving_least_squares(pc, search_radius=0.5, compute_normals=True)
    assert output.size() == 4942
    assert np.any(output.normals)


def test_moving_least_squares_alias():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output1 = pclpy.moving_least_squares(pc, search_radius=0.5, compute_normals=False)
    output2 = pclpy.mls(pc, search_radius=0.5, compute_normals=False)
    assert np.allclose(output1.xyz, output2.xyz)


def test_moving_least_squares_no_normals():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output = pclpy.moving_least_squares(pc, search_radius=0.5, compute_normals=False)
    assert output.size() == 4942
    assert not hasattr(output, "normals")


def test_moving_least_squares_no_normals_omp():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output = pclpy.moving_least_squares(pc, search_radius=0.5, compute_normals=False, num_threads=8)
    assert output.size() == 4942
    assert not hasattr(output, "normals")


def test_moving_least_squares_with_output():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output = pcl.PointCloud.PointNormal()
    pclpy.moving_least_squares(pc, search_radius=0.5, output_cloud=output)
    assert output.size() == 4942


def test_greedy_projection_triangulation_simple():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZRGBA")

    cloud_with_normals = pcl.PointCloud.PointNormal()
    pc.compute_normals(radius=0.15, output_cloud=cloud_with_normals, num_threads=8)

    ms = pcl.surface.GreedyProjectionTriangulation.PointNormal()

    pi = 3.141592

    triangles = pcl.PolygonMesh()
    ms.setSearchRadius(0.2)
    ms.setMu(2.5)
    ms.setMaximumNearestNeighbors(100)
    ms.setMaximumSurfaceAngle(pi / 4)
    ms.setMinimumAngle(pi / 18)
    ms.setMaximumAngle(2 * pi / 3)
    ms.setNormalConsistency(True)
    ms.setInputCloud(cloud_with_normals)
    tree2 = pcl.search.KdTree.PointNormal()
    ms.setSearchMethod(tree2)
    ms.reconstruct(triangles)

    assert len(triangles.polygons) == 148
