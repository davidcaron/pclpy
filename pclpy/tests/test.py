import pytest
import os
import numpy as np

import pclpy
from pclpy import pcl


@pytest.fixture
def xyz():
    a = np.random.ranf(30).reshape(-1, 3)
    p = pcl.PointCloud.PointXYZ.from_array(a)
    return p


def test_data(*args):
    return os.path.join("test_data", *args)


def test_hi():
    assert pcl.__doc__


def test_points(xyz):
    pt = xyz.points[0]
    pt.x = 2
    assert abs(pt.x - 2) < 0.000001


def test_point_indices():
    ind = pcl.PointIndices()
    ind.indices.append(3)
    ind.indices.append(4)
    assert list(ind.indices) == [3, 4]
    vectors_int = pcl.vectors.Int()
    vectors_int.append(5)
    vectors_int.append(6)
    vectors_int.append(7)
    ind.indices = vectors_int
    assert list(ind.indices) == [5, 6, 7]


def test_size_xyz(xyz):
    assert xyz.size() == 10


def test_enums():
    assert pcl.sample_consensus.SAC_RANSAC == 0
    assert pcl.sample_consensus.SAC_LMEDS == 1
    assert pcl.sample_consensus.SAC_MSAC == 2
    assert pcl.sample_consensus.SAC_RRANSAC == 3

    assert pcl.sample_consensus.SacModel.SACMODEL_PLANE == 0
    assert pcl.sample_consensus.SacModel.SACMODEL_LINE == 1
    assert pcl.sample_consensus.SacModel.SACMODEL_CIRCLE2D == 2
    assert pcl.sample_consensus.SacModel.SACMODEL_CIRCLE3D == 3


def test_make_kdtree():
    kdtree = pcl.kdtree.KdTreeFLANN.PointXYZ()
    assert kdtree
