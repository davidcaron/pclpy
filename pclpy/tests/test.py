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


# def test_points(xyz):
#     print(xyz.points)


#
# def test_voxel_grid_rgba(xyzrgba):
#     out = xyzrgba.voxel_grid(leaf_size=0.5)
#     assert out.size()
#     assert out.size() < xyzrgba.size()
#     assert isinstance(out, type(xyzrgba))
#
#
# def test_voxel_grid_xyz(xyz):
#     out = xyz.voxel_grid(leaf_size=0.5)
#     assert out.size()
#     assert out.size() < xyz.size()
#     assert isinstance(out, type(xyz))
#
#
# def test_approximate_voxel_grid_rgba(xyzrgba):
#     out = xyzrgba.approximate_voxel_grid(leaf_size=0.5)
#     assert out.size()
#     assert out.size() < xyzrgba.size()
#     assert isinstance(out, type(xyzrgba))
#
#
# def test_approximate_voxel_grid_xyz(xyz):
#     out = xyz.approximate_voxel_grid(leaf_size=0.5)
#     assert out.size()
#     assert out.size() < xyz.size()
#     assert isinstance(out, type(xyz))
#
#
# def test_sor_rgba(xyzrgba):
#     out = xyzrgba.sor(threshold=0.1, mean_k=2)
#     assert out.size()
#     assert out.size() < xyzrgba.size()
#     assert isinstance(out, type(xyzrgba))
#
#
# def test_sor_xyz(xyz):
#     out = xyz.sor(threshold=0.1, mean_k=2)
#     assert out.size()
#     assert out.size() < xyz.size()
#     assert isinstance(out, type(xyz))
#
#
# def test_extract_clusters_xyz(xyz):
#     clouds = xyz.extract_clusters(0.6, 2, 10)
#     assert clouds and clouds[0].size()
#
#
# def test_extract_clusters_xyzrgba(xyzrgba):
#     clouds = xyzrgba.extract_clusters(0.6, 2, 10)
#     assert clouds and clouds[0].size()
#
#
# def test_estimate_normals_xyz(xyz):
#     out = pcl.PointCloud.PointNormal()
#     pn = xyz.estimate_normals(out, 0.6)
#     assert pn.size()
#
#
# def test_estimate_normals_xyzrgba(xyzrgba):
#     out = pcl.PointCloud.PointNormal()
#     pn = xyzrgba.estimate_normals(out, 0.6)
#     assert pn.size()
#
#
# def test_convex_hull_xyz(xyz):
#     pn = xyz.convex_hull()
#     assert pn.size()
#     assert pn.size() < xyz.size()
#
#
# def test_convex_hull_xyzrgba(xyzrgba):
#     pn = xyzrgba.convex_hull()
#     assert pn.size()
#     assert pn.size() < xyzrgba.size()
#
#
# def test_concave_hull_xyz(xyz):
#     pn = xyz.concave_hull(0.8)
#     assert pn.size()
#     assert pn.size() < xyz.size()
#
#
# def test_concave_hull_xyzrgba(xyzrgba):
#     pn = xyzrgba.concave_hull(0.8)
#     assert pn.size()
#     assert pn.size() < xyzrgba.size()
#
#
# def test_crop_hull_xyz(xyz):
#     c = xyz.convex_hull()
#     pn = xyz.crop_hull(c)
#     assert pn.size()
#     assert pn.size() == xyz.size()
#
#
# def test_crop_hull_xyzrgba(xyzrgba):
#     c = xyzrgba.convex_hull()
#     pn = xyzrgba.crop_hull(c)
#     assert pn.size()
#     assert pn.size() == xyzrgba.size()


def test_io_las():
    pc = pclpy.io.read_las(test_data("street.las"))
    assert pc.size() == 1091656
    assert isinstance(pc, pcl.PointCloud.PointXYZRGBA)


def test_make_kdtree():
    kdtree = pcl.kdtree.KdTreeFLANN.PointXYZ()
    assert kdtree
