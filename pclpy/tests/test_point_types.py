import numpy as np
import pytest

from pclpy import pcl


def test_buffers_xyz():
    a = np.random.ranf(30).reshape(-1, 3)
    p = pcl.PointCloud.PointXYZ.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.xyz, a[:, :3])


def test_buffers_xyzi():
    a = np.random.ranf(40).reshape(-1, 4)
    p = pcl.PointCloud.PointXYZI.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.intensity, a[:, 3])
    assert np.allclose(p.xyz, a[:, :3])


def test_buffers_xyzl():
    a = (np.random.ranf(40) * 100).reshape(-1, 4)
    label = a[:, 3].astype("u4")
    p = pcl.PointCloud.PointXYZL.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.label, label)
    assert np.allclose(p.xyz, a[:, :3])


def test_buffers_label():
    a = (np.random.ranf(10) * 100).reshape(-1, 1)
    label = a[:, 0].astype("u4")
    p = pcl.PointCloud.Label.from_array(a)
    assert np.allclose(p.label, label)


def test_buffers_xyzrgba():
    xyz = np.random.ranf(30).reshape(-1, 3)
    rgb = (np.random.ranf(30) * 100).astype("u1").reshape(-1, 3)
    p = pcl.PointCloud.PointXYZRGBA.from_array(xyz, rgb)
    assert np.allclose(p.x, xyz[:, 0])
    assert np.allclose(p.y, xyz[:, 1])
    assert np.allclose(p.z, xyz[:, 2])
    assert np.allclose(p.xyz, xyz[:, :3])
    assert np.allclose(p.r, rgb[:, 0])
    assert np.allclose(p.g, rgb[:, 1])
    assert np.allclose(p.b, rgb[:, 2])
    assert np.allclose(p.a, np.full(xyz.shape[0], 255))
    assert np.allclose(p.rgb_reversed[:, ::-1], rgb)


def test_buffers_xyzrgb():
    xyz = np.random.ranf(30).reshape(-1, 3)
    rgb = (np.random.ranf(30) * 100).astype("u1").reshape(-1, 3)
    p = pcl.PointCloud.PointXYZRGB.from_array(xyz, rgb)
    assert np.allclose(p.x, xyz[:, 0])
    assert np.allclose(p.y, xyz[:, 1])
    assert np.allclose(p.z, xyz[:, 2])
    assert np.allclose(p.xyz, xyz[:, :3])
    assert np.allclose(p.r, rgb[:, 0])
    assert np.allclose(p.g, rgb[:, 1])
    assert np.allclose(p.b, rgb[:, 2])
    assert np.allclose(p.a, np.full(xyz.shape[0], 255))
    assert np.allclose(p.rgb_reversed[:, ::-1], rgb)


def test_buffers_xyzrgbl():
    xyz = np.random.ranf(40).reshape(-1, 4)
    label = xyz[:, 3].astype("u4")
    rgb = (np.random.ranf(30) * 100).astype("u1").reshape(-1, 3)
    p = pcl.PointCloud.PointXYZRGBL.from_array(xyz, rgb)
    assert np.allclose(p.x, xyz[:, 0])
    assert np.allclose(p.y, xyz[:, 1])
    assert np.allclose(p.z, xyz[:, 2])
    assert np.allclose(p.xyz, xyz[:, :3])
    assert np.allclose(p.label, label)
    assert np.allclose(p.r, rgb[:, 0])
    assert np.allclose(p.g, rgb[:, 1])
    assert np.allclose(p.b, rgb[:, 2])
    assert np.allclose(p.a, np.full(xyz.shape[0], 255))
    assert np.allclose(p.rgb_reversed[:, ::-1], rgb)


def test_buffers_xyzhsv():
    a = (np.random.ranf(60) * 100).reshape(-1, 6)
    p = pcl.PointCloud.PointXYZHSV.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.xyz, a[:, :3])
    assert np.allclose(p.h, a[:, 3])
    assert np.allclose(p.s, a[:, 4])
    assert np.allclose(p.v, a[:, 5])
    assert np.allclose(p.hsv, a[:, 3:6])


def test_buffers_pointnormal():
    a = np.random.ranf(70).reshape(-1, 7)
    p = pcl.PointCloud.PointNormal.from_array(a)
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
    p = pcl.PointCloud.Normal.from_array(a)
    assert np.allclose(p.normal_x, a[:, 0])
    assert np.allclose(p.normal_y, a[:, 1])
    assert np.allclose(p.normal_z, a[:, 2])
    assert np.allclose(p.curvature, a[:, 3])
    assert np.allclose(p.normals, a[:, :3])
