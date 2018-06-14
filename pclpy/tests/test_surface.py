import pytest
import os
import numpy as np
import laspy

from pclpy import pcl
import pclpy


def test_data(*args):
    return os.path.join("test_data", *args)


def test_moving_least_squares_normals():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output = pclpy.moving_least_squares(pc, search_radius=0.5, compute_normals=True)
    assert output.size() == 4942
    assert np.any(output.normals)


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
