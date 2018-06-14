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
