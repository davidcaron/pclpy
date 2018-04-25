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


def test_buffers_principalcurvatures():
    a = np.random.ranf(50).reshape(-1, 5)
    p = pcl.PointCloud.PrincipalCurvatures.from_array(a)
    assert np.allclose(p.principal_curvature_x, a[:, 0])
    assert np.allclose(p.principal_curvature_y, a[:, 1])
    assert np.allclose(p.principal_curvature_z, a[:, 2])
    assert np.allclose(p.principal_curvature, a[:, :3])
    assert np.allclose(p.pc1, a[:, 3])
    assert np.allclose(p.pc2, a[:, 4])


def test_buffers_pfhsignature125():
    a = np.random.ranf(1250).reshape(10, 125)
    p = pcl.PointCloud.PFHSignature125.from_array(a)
    assert np.allclose(p.histogram, a)
    assert pcl.point_types.PFHSignature125.descriptorSize() == 125


def test_buffers_pfhsignature250():
    a = np.random.ranf(2500).reshape(10, 250)
    p = pcl.PointCloud.PFHRGBSignature250.from_array(a)
    assert np.allclose(p.histogram, a)
    assert pcl.point_types.PFHRGBSignature250.descriptorSize() == 250


def test_buffers_ppfsignature():
    a = np.random.ranf(50).reshape(-1, 5)
    p = pcl.PointCloud.PPFSignature.from_array(a)
    assert np.allclose(p.f1, a[:, 0])
    assert np.allclose(p.f2, a[:, 1])
    assert np.allclose(p.f3, a[:, 2])
    assert np.allclose(p.f4, a[:, 3])
    assert np.allclose(p.alpha_m, a[:, 4])


def test_buffers_cppfsignature():
    a = np.random.ranf(110).reshape(-1, 11)
    p = pcl.PointCloud.CPPFSignature.from_array(a)
    assert np.allclose(p.f1, a[:, 0])
    assert np.allclose(p.f2, a[:, 1])
    assert np.allclose(p.f3, a[:, 2])
    assert np.allclose(p.f4, a[:, 3])
    assert np.allclose(p.f5, a[:, 4])
    assert np.allclose(p.f6, a[:, 5])
    assert np.allclose(p.f7, a[:, 6])
    assert np.allclose(p.f8, a[:, 7])
    assert np.allclose(p.f9, a[:, 8])
    assert np.allclose(p.f10, a[:, 9])
    assert np.allclose(p.alpha_m, a[:, 10])


def ppfrgbsignature():
    a = np.random.ranf(80).reshape(-1, 8)
    p = pcl.PointCloud.PPFRGBSignature.from_array(a)
    assert np.allclose(p.f1, a[:, 0])
    assert np.allclose(p.f2, a[:, 1])
    assert np.allclose(p.f3, a[:, 2])
    assert np.allclose(p.f4, a[:, 3])
    assert np.allclose(p.r_ratio, a[:, 4])
    assert np.allclose(p.g_ratio, a[:, 5])
    assert np.allclose(p.b_ratio, a[:, 6])
    assert np.allclose(p.alpha_m, a[:, 7])


def test_buffers_normalbasedsignature12():
    a = np.random.ranf(120).reshape(10, 12)
    p = pcl.PointCloud.NormalBasedSignature12.from_array(a)
    assert np.allclose(p.values, a)


def test_buffers_fpfhsignature33():
    a = np.random.ranf(330).reshape(10, 33)
    p = pcl.PointCloud.FPFHSignature33.from_array(a)
    assert np.allclose(p.histogram, a)
    assert pcl.point_types.FPFHSignature33.descriptorSize() == 33


def test_buffers_vfhsignature308():
    a = np.random.ranf(3080).reshape(10, 308)
    p = pcl.PointCloud.VFHSignature308.from_array(a)
    assert np.allclose(p.histogram, a)
    assert pcl.point_types.VFHSignature308.descriptorSize() == 308


def test_buffers_grsdsignature21():
    a = np.random.ranf(210).reshape(10, 21)
    p = pcl.PointCloud.GRSDSignature21.from_array(a)
    assert np.allclose(p.histogram, a)
    assert pcl.point_types.GRSDSignature21.descriptorSize() == 21


def test_buffers_esfsignature640():
    a = np.random.ranf(6400).reshape(10, 640)
    p = pcl.PointCloud.ESFSignature640.from_array(a)
    assert np.allclose(p.histogram, a)
    assert pcl.point_types.ESFSignature640.descriptorSize() == 640


def test_buffers_brisksignature512():
    a = np.random.ranf(660).reshape(10, 66) * 100
    descriptors = a[:, 2:].astype("u1")
    p = pcl.PointCloud.BRISKSignature512.from_array(a)
    assert np.allclose(p.scale, a[:, 0])
    assert np.allclose(p.orientation, a[:, 1])
    assert np.allclose(p.descriptor, descriptors)
    assert pcl.point_types.BRISKSignature512.descriptorSize() == 64


def test_buffers_narf36():
    a = np.random.ranf(420).reshape(10, 42)
    p = pcl.PointCloud.Narf36.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.roll, a[:, 3])
    assert np.allclose(p.pitch, a[:, 4])
    assert np.allclose(p.yaw, a[:, 5])
    assert np.allclose(p.descriptor, a[:, 6:])
    assert pcl.point_types.Narf36.descriptorSize() == 36


def test_buffers_intensitygradient():
    a = np.random.ranf(30).reshape(-1, 3)
    p = pcl.PointCloud.IntensityGradient.from_array(a)
    assert np.allclose(p.gradient, a[:, :3])
    assert np.allclose(p.gradient_x, a[:, 0])
    assert np.allclose(p.gradient_y, a[:, 1])
    assert np.allclose(p.gradient_z, a[:, 2])


def test_buffers_pointwithscale():
    a = np.random.ranf(70).reshape(-1, 7)
    octave = a[:, 6].astype("i4")
    p = pcl.PointCloud.PointWithScale.from_array(a)
    assert np.allclose(p.x, a[:, 0])
    assert np.allclose(p.y, a[:, 1])
    assert np.allclose(p.z, a[:, 2])
    assert np.allclose(p.xyz, a[:, :3])
    assert np.allclose(p.scale, a[:, 3])
    assert np.allclose(p.size, a[:, 3])
    assert np.allclose(p.angle, a[:, 4])
    assert np.allclose(p.response, a[:, 5])
    assert np.allclose(p.octave, octave)


def test_buffers_pointsurfel():
    xyz = np.random.ranf(90).reshape(-1, 9)
    rgb = (np.random.ranf(30) * 100).astype("u1").reshape(-1, 3)
    p = pcl.PointCloud.PointSurfel.from_array(xyz, rgb)
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
    assert np.allclose(p.radius, xyz[:, 6])
    assert np.allclose(p.confidence, xyz[:, 7])
    assert np.allclose(p.curvature, xyz[:, 8])


def test_buffers_shapecontext1980():
    a = np.random.ranf(10 * (1980 + 9)).reshape(10, -1)
    p = pcl.PointCloud.ShapeContext1980.from_array(a)
    assert np.allclose(p.descriptor, a[:, :1980])
    assert np.allclose(p.rf, a[:, 1980:])
    assert pcl.point_types.ShapeContext1980.descriptorSize() == 1980


def test_buffers_uniqueshapecontext1960():
    a = np.random.ranf(10 * (1960 + 9)).reshape(10, -1)
    p = pcl.PointCloud.UniqueShapeContext1960.from_array(a)
    assert np.allclose(p.descriptor, a[:, :1960])
    assert np.allclose(p.rf, a[:, 1960:])
    assert pcl.point_types.UniqueShapeContext1960.descriptorSize() == 1960


def test_buffers_shot352():
    a = np.random.ranf(10 * (352 + 9)).reshape(10, -1)
    p = pcl.PointCloud.SHOT352.from_array(a)
    assert np.allclose(p.descriptor, a[:, :352])
    assert np.allclose(p.rf, a[:, 352:])
    assert pcl.point_types.SHOT352.descriptorSize() == 352


def test_buffers_shot1344():
    a = np.random.ranf(10 * (1344 + 9)).reshape(10, -1)
    p = pcl.PointCloud.SHOT1344.from_array(a)
    assert np.allclose(p.descriptor, a[:, :1344])
    assert np.allclose(p.rf, a[:, 1344:])
    assert pcl.point_types.SHOT1344.descriptorSize() == 1344


def test_buffers_uv():
    a = np.random.ranf(20).reshape(-1, 2)
    p = pcl.PointCloud.PointUV.from_array(a)
    assert np.allclose(p.u, a[:, 0])
    assert np.allclose(p.v, a[:, 1])


def test_buffers_referenceframe():
    a = np.random.ranf(10 * 9).reshape(10, -1)
    p = pcl.PointCloud.ReferenceFrame.from_array(a)
    assert np.allclose(p.rf, a)
    assert np.allclose(p.x_axis, a[:, :3])
    assert np.allclose(p.y_axis, a[:, 3:6])
    assert np.allclose(p.z_axis, a[:, 6:])


def test_buffers_pointdem():
    xyz = np.random.ranf(60).reshape(-1, 6)
    p = pcl.PointCloud.PointDEM.from_array(xyz)
    assert np.allclose(p.x, xyz[:, 0])
    assert np.allclose(p.y, xyz[:, 1])
    assert np.allclose(p.z, xyz[:, 2])
    assert np.allclose(p.xyz, xyz[:, :3])
    assert np.allclose(p.intensity, xyz[:, 3])
    assert np.allclose(p.intensity_variance, xyz[:, 4])
    assert np.allclose(p.height_variance, xyz[:, 5])
