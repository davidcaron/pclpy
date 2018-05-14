import pytest
import os
import numpy as np
import laspy
import types

from pclpy import pcl
import pclpy


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


def test_initialize_everything():
    modules = [m for m in dir(pcl) if not m.startswith("__") and m[0].islower()]

    for module_name in modules:
        module = getattr(pcl, module_name)
        sub_modules = [c for c in dir(module) if not c.startswith("__")]
        for name in sub_modules:
            class_ = getattr(module, name)
            if isinstance(class_, types.ModuleType):
                print(class_, type(class_))


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
