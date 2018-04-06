import numpy as np
import pytest

from pclpy import pcl


def test_buffers_xyz():
    a = np.random.ranf(30).reshape(-1, 3)
    p = pcl.PointCloudXYZ.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.xyz, a[:, :3])


def test_buffers_xyzi():
    a = np.random.ranf(40).reshape(-1, 4)
    p = pcl.PointCloudXYZI.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.intensity, a[:, 3])
    assert np.allclose(p.xyz, a[:, :3])


def test_buffers_xyzl():
    a = (np.random.ranf(40) * 100).reshape(-1, 4)
    labels = a[:, 3].astype("u4")
    p = pcl.PointCloudXYZL.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.labels, labels)
    assert np.allclose(p.xyz, a[:, :3])


def test_buffers_pointnormal():
    a = np.random.ranf(70).reshape(-1, 7)
    p = pcl.PointCloudPointNormal.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.xyz, a[:, :3])
    assert np.allclose(p.normal_x, a[:, 3])
    assert np.allclose(p.normal_y, a[:, 4])
    assert np.allclose(p.normal_z, a[:, 5])
    assert np.allclose(p.normals, a[:, 3:6])
    assert np.allclose(p.curvature, a[:, 6])


def test_buffers_normal():
    a = np.random.ranf(40).reshape(-1, 4)
    p = pcl.PointCloudNormal.from_array(a)
    assert np.allclose(p.normal_x, a[:, 0])
    assert np.allclose(p.normal_y, a[:, 1])
    assert np.allclose(p.normal_z, a[:, 2])
    assert np.allclose(p.curvature, a[:, 3])
    assert np.allclose(p.normals, a[:, :3])


def test_buffers_xyzrgba():
    xyz = np.random.ranf(30).reshape(-1, 3)
    rgb = (np.random.ranf(30) * 100).astype("u1").reshape(-1, 3)
    p = pcl.PointCloudXYZRGBA.from_array(xyz, rgb)
    assert np.allclose(p.x, xyz[:, 0])
    assert np.allclose(p.y, xyz[:, 1])
    assert np.allclose(p.z, xyz[:, 2])
    assert np.allclose(p.xyz, xyz[:, :3])
    assert np.allclose(p.r, rgb[:, 0])
    assert np.allclose(p.g, rgb[:, 1])
    assert np.allclose(p.b, rgb[:, 2])
    assert np.allclose(p.rgb_reversed[:, ::-1], rgb)
