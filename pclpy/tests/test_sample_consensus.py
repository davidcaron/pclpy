import pytest
import os
import numpy as np
from math import cos, sin, pi

from pclpy import pcl
import pclpy


def test_data(*args):
    return os.path.join("test_data", *args)


def test_fit_line():
    line = np.array([(1, 2, 3), (2, 4, 6), (3, 7, 9), (5, 10, 15)])
    pc = pcl.PointCloud.PointXYZ(line)
    inliers, coefficients = pclpy.fit(pc, "line", distance=0.1)
    assert len(inliers.indices) == 3
    assert np.allclose(coefficients.values, pcl.vectors.Float([2.66667, 5.33333, 8, 0.267261, 0.534522, 0.801784]))


def test_fit_cylinder():
    points = np.array([(cos(a), sin(a), z) for z in np.linspace(0, 5, num=10) for a in np.linspace(0, 2 * pi, num=20)])
    points = np.vstack((np.array([10, 10, 10]), points))
    pc = pcl.PointCloud.PointXYZ(points)
    inliers, coefficients = pclpy.fit(pc, "circle2d", distance=0.01, indices=np.arange(100))
    assert 0 not in inliers.indices
    assert len(inliers.indices) == 99
    assert np.allclose(coefficients.values, [0., 0., 1.], atol=0.00001)
