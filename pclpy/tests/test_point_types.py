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


def test_buffers_xy():
    a = np.random.ranf(20).reshape(-1, 2)
    p = pcl.PointCloud.PointXY.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])


def test_buffers_interestpoint():
    a = np.random.ranf(40).reshape(-1, 4)
    p = pcl.PointCloud.InterestPoint.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.strength, a[:, 3])
    assert np.allclose(p.xyz, a[:, :3])


def test_buffers_axis():
    a = np.random.ranf(30).reshape(-1, 3)
    p = pcl.PointCloud.Axis.from_array(a)
    assert np.allclose(p.normal_x, a[:, 0])
    assert np.allclose(p.normal_y, a[:, 1])
    assert np.allclose(p.normal_z, a[:, 2])
    assert np.allclose(p.normals, a[:, :3])


def test_buffers_normal():
    a = np.random.ranf(40).reshape(-1, 4)
    p = pcl.PointCloud.Normal.from_array(a)
    assert np.allclose(p.normal_x, a[:, 0])
    assert np.allclose(p.normal_y, a[:, 1])
    assert np.allclose(p.normal_z, a[:, 2])
    assert np.allclose(p.curvature, a[:, 3])
    assert np.allclose(p.normals, a[:, :3])


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


def test_buffers_pointxyzrgbnormal():
    xyz = np.random.ranf(70).reshape(-1, 7)
    rgb = (np.random.ranf(30) * 100).astype("u1").reshape(-1, 3)
    p = pcl.PointCloud.PointXYZRGBNormal.from_array(xyz, rgb)
    assert np.allclose(p.x, xyz[:, 0])
    assert np.allclose(p.y, xyz[:, 1])
    assert np.allclose(p.z, xyz[:, 2])
    assert np.allclose(p.xyz, xyz[:, :3])
    assert np.allclose(p.normal_x, xyz[:, 3])
    assert np.allclose(p.normal_y, xyz[:, 4])
    assert np.allclose(p.normal_z, xyz[:, 5])
    assert np.allclose(p.normals, xyz[:, 3:6])
    assert np.allclose(p.r, rgb[:, 0])
    assert np.allclose(p.g, rgb[:, 1])
    assert np.allclose(p.b, rgb[:, 2])
    assert np.allclose(p.a, np.full(xyz.shape[0], 255))
    assert np.allclose(p.rgb_reversed[:, ::-1], rgb)
    assert np.allclose(p.curvature, xyz[:, 6])


def test_buffers_xyzinormal():
    a = np.random.ranf(80).reshape(-1, 8)
    p = pcl.PointCloud.PointXYZINormal.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.xyz, a[:, :3])
    assert np.allclose(p.intensity, a[:, 3])
    assert np.allclose(p.normal_x, a[:, 4])
    assert np.allclose(p.normal_y, a[:, 5])
    assert np.allclose(p.normal_z, a[:, 6])
    assert np.allclose(p.normals, a[:, 4:7])
    assert np.allclose(p.curvature, a[:, 7])


def test_buffers_xyzlnormal():
    a = np.random.ranf(80).reshape(-1, 8)
    label = a[:, 3].astype("u4")
    p = pcl.PointCloud.PointXYZLNormal.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.xyz, a[:, :3])
    assert np.allclose(p.label, label)
    assert np.allclose(p.normal_x, a[:, 4])
    assert np.allclose(p.normal_y, a[:, 5])
    assert np.allclose(p.normal_z, a[:, 6])
    assert np.allclose(p.normals, a[:, 4:7])
    assert np.allclose(p.curvature, a[:, 7])


def test_buffers_pointwithrange():
    a = np.random.ranf(40).reshape(-1, 4)
    p = pcl.PointCloud.PointWithRange.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.range, a[:, 3])
    assert np.allclose(p.xyz, a[:, :3])


def test_buffers_pointwithviewpoint():
    a = np.random.ranf(60).reshape(-1, 6)
    p = pcl.PointCloud.PointWithViewpoint.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.xyz, a[:, :3])
    assert np.allclose(p.vp_x, a[:, 3])
    assert np.allclose(p.vp_y, a[:, 4])
    assert np.allclose(p.vp_z, a[:, 5])
    assert np.allclose(p.vp, a[:, 3:6])


def test_buffers_momentinvariants():
    a = np.random.ranf(30).reshape(-1, 3)
    p = pcl.PointCloud.MomentInvariants.from_array(a)
    assert np.allclose(p.j1, a[:, 0])
    assert np.allclose(p.j2, a[:, 1])
    assert np.allclose(p.j3, a[:, 2])


def test_buffers_principalradiirsd():
    a = np.random.ranf(20).reshape(-1, 2)
    p = pcl.PointCloud.PrincipalRadiiRSD.from_array(a)
    assert np.allclose(p.r_min, a[:, 0])
    assert np.allclose(p.r_max, a[:, 1])


def test_buffers_boundary():
    a = np.random.ranf(10).reshape(-1, 1)
    boundary_point = a.astype("u1")
    p = pcl.PointCloud.Boundary.from_array(a)
    assert np.allclose(p.boundary_point, boundary_point)
