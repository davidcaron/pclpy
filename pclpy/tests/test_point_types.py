import numpy as np
import pytest

from pclpy import pcl


def test_buffers_xyz():
    a = np.random.ranf(30).reshape(-1, 3)
    for p in [pcl.PointCloud.PointXYZ(a), pcl.PointCloud.PointXYZ.from_array(a)]:
        assert np.allclose(p.x, a[:, 0])
        assert np.allclose(p.y, a[:, 1])
        assert np.allclose(p.z, a[:, 2])
        assert np.allclose(p.xyz, a[:, :3])


def test_buffers_xyzi():
    a = np.random.ranf(40).reshape(-1, 4)
    for p in [pcl.PointCloud.PointXYZI(a), pcl.PointCloud.PointXYZI.from_array(a)]:
        assert np.allclose(p.x, a[:, 0])
        assert np.allclose(p.y, a[:, 1])
        assert np.allclose(p.z, a[:, 2])
        assert np.allclose(p.intensity, a[:, 3])
        assert np.allclose(p.xyz, a[:, :3])


def test_buffers_xyzl():
    a = (np.random.ranf(40) * 100).reshape(-1, 4)
    label = a[:, 3].astype("u4")
    for p in [pcl.PointCloud.PointXYZL(a), pcl.PointCloud.PointXYZL.from_array(a)]:
        assert np.allclose(p.x, a[:, 0])
        assert np.allclose(p.y, a[:, 1])
        assert np.allclose(p.z, a[:, 2])
        assert np.allclose(p.label, label)
        assert np.allclose(p.xyz, a[:, :3])


def test_buffers_label():
    a = (np.random.ranf(10) * 100).reshape(-1, 1)
    label = a[:, 0].astype("u4")
    for p in [pcl.PointCloud.Label(a), pcl.PointCloud.Label.from_array(a)]:
        assert np.allclose(p.label, label)


def test_buffers_xyzrgba():
    xyz = np.random.ranf(30).reshape(-1, 3)
    rgb = (np.random.ranf(30) * 100).astype("u1").reshape(-1, 3)
    for p in [pcl.PointCloud.PointXYZRGBA(xyz, rgb), pcl.PointCloud.PointXYZRGBA.from_array(xyz, rgb)]:
        assert np.allclose(p.x, xyz[:, 0])
        assert np.allclose(p.y, xyz[:, 1])
        assert np.allclose(p.z, xyz[:, 2])
        assert np.allclose(p.xyz, xyz[:, :3])
        assert np.allclose(p.r, rgb[:, 0])
        assert np.allclose(p.g, rgb[:, 1])
        assert np.allclose(p.b, rgb[:, 2])
        alpha = np.full(xyz.shape[0], 255).reshape(-1, 1)
        assert np.allclose(p.a, alpha)
        assert np.allclose(p.rgb, rgb)
        assert np.allclose(p.argb, np.hstack([alpha, rgb]))


def test_buffers_xyzrgb():
    xyz = np.random.ranf(30).reshape(-1, 3)
    rgb = (np.random.ranf(30) * 100).astype("u1").reshape(-1, 3)
    for p in [pcl.PointCloud.PointXYZRGB(xyz, rgb), pcl.PointCloud.PointXYZRGB.from_array(xyz, rgb)]:
        assert np.allclose(p.x, xyz[:, 0])
        assert np.allclose(p.y, xyz[:, 1])
        assert np.allclose(p.z, xyz[:, 2])
        assert np.allclose(p.xyz, xyz[:, :3])
        assert np.allclose(p.r, rgb[:, 0])
        assert np.allclose(p.g, rgb[:, 1])
        assert np.allclose(p.b, rgb[:, 2])
        alpha = np.full(xyz.shape[0], 255).reshape(-1, 1)
        assert np.allclose(p.a, alpha)
        assert np.allclose(p.rgb, rgb)
        assert np.allclose(p.argb, np.hstack([alpha, rgb]))


def test_buffers_xyzrgbl():
    xyz = np.random.ranf(40).reshape(-1, 4)
    label = xyz[:, 3].astype("u4")
    rgb = (np.random.ranf(30) * 100).astype("u1").reshape(-1, 3)
    for p in [pcl.PointCloud.PointXYZRGBL(xyz, rgb), pcl.PointCloud.PointXYZRGBL.from_array(xyz, rgb)]:
        assert np.allclose(p.x, xyz[:, 0])
        assert np.allclose(p.y, xyz[:, 1])
        assert np.allclose(p.z, xyz[:, 2])
        assert np.allclose(p.xyz, xyz[:, :3])
        assert np.allclose(p.label, label)
        assert np.allclose(p.r, rgb[:, 0])
        assert np.allclose(p.g, rgb[:, 1])
        assert np.allclose(p.b, rgb[:, 2])
        alpha = np.full(xyz.shape[0], 255).reshape(-1, 1)
        assert np.allclose(p.a, alpha)
        assert np.allclose(p.rgb, rgb)
        assert np.allclose(p.argb, np.hstack([alpha, rgb]))


def test_buffers_xyzhsv():
    a = (np.random.ranf(60) * 100).reshape(-1, 6)
    for p in [pcl.PointCloud.PointXYZHSV(a), pcl.PointCloud.PointXYZHSV.from_array(a)]:
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
    for p in [pcl.PointCloud.PointXY(a), pcl.PointCloud.PointXY.from_array(a)]:
        assert np.allclose(p.x, a[:, 0])
        assert np.allclose(p.y, a[:, 1])


def test_buffers_interestpoint():
    a = np.random.ranf(40).reshape(-1, 4)
    for p in [pcl.PointCloud.InterestPoint(a), pcl.PointCloud.InterestPoint.from_array(a)]:
        assert np.allclose(p.x, a[:, 0])
        assert np.allclose(p.y, a[:, 1])
        assert np.allclose(p.z, a[:, 2])
        assert np.allclose(p.strength, a[:, 3])
        assert np.allclose(p.xyz, a[:, :3])


def test_buffers_axis():
    a = np.random.ranf(30).reshape(-1, 3)
    for p in [pcl.PointCloud.Axis(a), pcl.PointCloud.Axis.from_array(a)]:
        assert np.allclose(p.normal_x, a[:, 0])
        assert np.allclose(p.normal_y, a[:, 1])
        assert np.allclose(p.normal_z, a[:, 2])
        assert np.allclose(p.normals, a[:, :3])


def test_buffers_normal():
    a = np.random.ranf(40).reshape(-1, 4)
    for p in [pcl.PointCloud.Normal(a), pcl.PointCloud.Normal.from_array(a)]:
        assert np.allclose(p.normal_x, a[:, 0])
        assert np.allclose(p.normal_y, a[:, 1])
        assert np.allclose(p.normal_z, a[:, 2])
        assert np.allclose(p.curvature, a[:, 3])
        assert np.allclose(p.normals, a[:, :3])


def test_buffers_pointnormal():
    a = np.random.ranf(70).reshape(-1, 7)
    for p in [pcl.PointCloud.PointNormal(a), pcl.PointCloud.PointNormal.from_array(a)]:
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
    for p in [pcl.PointCloud.PointXYZRGBNormal(xyz, rgb), pcl.PointCloud.PointXYZRGBNormal.from_array(xyz, rgb)]:
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
        alpha = np.full(xyz.shape[0], 255).reshape(-1, 1)
        assert np.allclose(p.a, alpha)
        assert np.allclose(p.rgb, rgb)
        assert np.allclose(p.argb, np.hstack([alpha, rgb]))
        assert np.allclose(p.curvature, xyz[:, 6])


def test_buffers_xyzinormal():
    a = np.random.ranf(80).reshape(-1, 8)
    for p in [pcl.PointCloud.PointXYZINormal(a), pcl.PointCloud.PointXYZINormal.from_array(a)]:
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
    for p in [pcl.PointCloud.PointXYZLNormal(a), pcl.PointCloud.PointXYZLNormal.from_array(a)]:
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
    for p in [pcl.PointCloud.PointWithRange(a), pcl.PointCloud.PointWithRange.from_array(a)]:
        assert np.allclose(p.x, a[:, 0])
        assert np.allclose(p.y, a[:, 1])
        assert np.allclose(p.z, a[:, 2])
        assert np.allclose(p.range, a[:, 3])
        assert np.allclose(p.xyz, a[:, :3])


def test_buffers_pointwithviewpoint():
    a = np.random.ranf(60).reshape(-1, 6)
    for p in [pcl.PointCloud.PointWithViewpoint(a), pcl.PointCloud.PointWithViewpoint.from_array(a)]:
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
    for p in [pcl.PointCloud.MomentInvariants(a), pcl.PointCloud.MomentInvariants.from_array(a)]:
        assert np.allclose(p.j1, a[:, 0])
        assert np.allclose(p.j2, a[:, 1])
        assert np.allclose(p.j3, a[:, 2])


def test_buffers_principalradiirsd():
    a = np.random.ranf(20).reshape(-1, 2)
    for p in [pcl.PointCloud.PrincipalRadiiRSD(a), pcl.PointCloud.PrincipalRadiiRSD.from_array(a)]:
        assert np.allclose(p.r_min, a[:, 0])
        assert np.allclose(p.r_max, a[:, 1])


def test_buffers_boundary():
    a = np.random.ranf(10).reshape(-1, 1)
    boundary_point = a.astype("u1")
    for p in [pcl.PointCloud.Boundary(a), pcl.PointCloud.Boundary.from_array(a)]:
        assert np.allclose(p.boundary_point, boundary_point)


def test_buffers_principalcurvatures():
    a = np.random.ranf(50).reshape(-1, 5)
    for p in [pcl.PointCloud.PrincipalCurvatures(a), pcl.PointCloud.PrincipalCurvatures.from_array(a)]:
        assert np.allclose(p.principal_curvature_x, a[:, 0])
        assert np.allclose(p.principal_curvature_y, a[:, 1])
        assert np.allclose(p.principal_curvature_z, a[:, 2])
        assert np.allclose(p.principal_curvature, a[:, :3])
        assert np.allclose(p.pc1, a[:, 3])
        assert np.allclose(p.pc2, a[:, 4])


def test_buffers_pfhsignature125():
    a = np.random.ranf(1250).reshape(10, 125)
    for p in [pcl.PointCloud.PFHSignature125(a), pcl.PointCloud.PFHSignature125.from_array(a)]:
        assert np.allclose(p.histogram, a)
        assert pcl.point_types.PFHSignature125.descriptorSize() == 125


def test_buffers_pfhsignature250():
    a = np.random.ranf(2500).reshape(10, 250)
    for p in [pcl.PointCloud.PFHRGBSignature250(a), pcl.PointCloud.PFHRGBSignature250.from_array(a)]:
        assert np.allclose(p.histogram, a)
        assert pcl.point_types.PFHRGBSignature250.descriptorSize() == 250


def test_buffers_ppfsignature():
    a = np.random.ranf(50).reshape(-1, 5)
    for p in [pcl.PointCloud.PPFSignature(a), pcl.PointCloud.PPFSignature.from_array(a)]:
        assert np.allclose(p.f1, a[:, 0])
        assert np.allclose(p.f2, a[:, 1])
        assert np.allclose(p.f3, a[:, 2])
        assert np.allclose(p.f4, a[:, 3])
        assert np.allclose(p.alpha_m, a[:, 4])


def test_buffers_cppfsignature():
    a = np.random.ranf(110).reshape(-1, 11)
    for p in [pcl.PointCloud.CPPFSignature(a), pcl.PointCloud.CPPFSignature.from_array(a)]:
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
    for p in [pcl.PointCloud.PPFRGBSignature(a), pcl.PointCloud.PPFRGBSignature.from_array(a)]:
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
    for p in [pcl.PointCloud.NormalBasedSignature12(a), pcl.PointCloud.NormalBasedSignature12.from_array(a)]:
        assert np.allclose(p.values, a)


def test_buffers_fpfhsignature33():
    a = np.random.ranf(330).reshape(10, 33)
    for p in [pcl.PointCloud.FPFHSignature33(a), pcl.PointCloud.FPFHSignature33.from_array(a)]:
        assert np.allclose(p.histogram, a)
        assert pcl.point_types.FPFHSignature33.descriptorSize() == 33


def test_buffers_vfhsignature308():
    a = np.random.ranf(3080).reshape(10, 308)
    for p in [pcl.PointCloud.VFHSignature308(a), pcl.PointCloud.VFHSignature308.from_array(a)]:
        assert np.allclose(p.histogram, a)
        assert pcl.point_types.VFHSignature308.descriptorSize() == 308


def test_buffers_gasdsignature512():
    a = np.random.ranf(5120).reshape(10, 512)
    for p in [pcl.PointCloud.GASDSignature512(a), pcl.PointCloud.GASDSignature512.from_array(a)]:
        assert np.allclose(p.histogram, a)
        assert pcl.point_types.GASDSignature512.descriptorSize() == 512


def test_buffers_gasdsignature984():
    a = np.random.ranf(9840).reshape(10, 984)
    for p in [pcl.PointCloud.GASDSignature984(a), pcl.PointCloud.GASDSignature984.from_array(a)]:
        assert np.allclose(p.histogram, a)
        assert pcl.point_types.GASDSignature984.descriptorSize() == 984


def test_buffers_gasdsignature7992():
    a = np.random.ranf(79920).reshape(10, 7992)
    for p in [pcl.PointCloud.GASDSignature7992(a), pcl.PointCloud.GASDSignature7992.from_array(a)]:
        assert np.allclose(p.histogram, a)
        assert pcl.point_types.GASDSignature7992.descriptorSize() == 7992


def test_buffers_grsdsignature21():
    a = np.random.ranf(210).reshape(10, 21)
    for p in [pcl.PointCloud.GRSDSignature21(a), pcl.PointCloud.GRSDSignature21.from_array(a)]:
        assert np.allclose(p.histogram, a)
        assert pcl.point_types.GRSDSignature21.descriptorSize() == 21


def test_buffers_esfsignature640():
    a = np.random.ranf(6400).reshape(10, 640)
    for p in [pcl.PointCloud.ESFSignature640(a), pcl.PointCloud.ESFSignature640.from_array(a)]:
        assert np.allclose(p.histogram, a)
        assert pcl.point_types.ESFSignature640.descriptorSize() == 640


def test_buffers_brisksignature512():
    a = np.random.ranf(660).reshape(10, 66) * 100
    descriptors = a[:, 2:].astype("u1")
    for p in [pcl.PointCloud.BRISKSignature512(a), pcl.PointCloud.BRISKSignature512.from_array(a)]:
        assert np.allclose(p.scale, a[:, 0])
        assert np.allclose(p.orientation, a[:, 1])
        assert np.allclose(p.descriptor, descriptors)
        assert pcl.point_types.BRISKSignature512.descriptorSize() == 64


def test_buffers_narf36():
    a = np.random.ranf(420).reshape(10, 42)
    for p in [pcl.PointCloud.Narf36(a), pcl.PointCloud.Narf36.from_array(a)]:
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
    for p in [pcl.PointCloud.IntensityGradient(a), pcl.PointCloud.IntensityGradient.from_array(a)]:
        assert np.allclose(p.gradient, a[:, :3])
        assert np.allclose(p.gradient_x, a[:, 0])
        assert np.allclose(p.gradient_y, a[:, 1])
        assert np.allclose(p.gradient_z, a[:, 2])


def test_buffers_pointwithscale():
    a = np.random.ranf(70).reshape(-1, 7)
    octave = a[:, 6].astype("i4")
    for p in [pcl.PointCloud.PointWithScale(a), pcl.PointCloud.PointWithScale.from_array(a)]:
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
    for p in [pcl.PointCloud.PointSurfel(xyz, rgb), pcl.PointCloud.PointSurfel.from_array(xyz, rgb)]:
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
        alpha = np.full(xyz.shape[0], 255).reshape(-1, 1)
        assert np.allclose(p.a, alpha)
        assert np.allclose(p.rgb, rgb)
        assert np.allclose(p.argb, np.hstack([alpha, rgb]))
        assert np.allclose(p.radius, xyz[:, 6])
        assert np.allclose(p.confidence, xyz[:, 7])
        assert np.allclose(p.curvature, xyz[:, 8])


def test_buffers_shapecontext1980():
    a = np.random.ranf(10 * (1980 + 9)).reshape(10, -1)
    for p in [pcl.PointCloud.ShapeContext1980(a), pcl.PointCloud.ShapeContext1980.from_array(a)]:
        assert np.allclose(p.descriptor, a[:, :1980])
        assert np.allclose(p.rf, a[:, 1980:])
        assert pcl.point_types.ShapeContext1980.descriptorSize() == 1980


def test_buffers_uniqueshapecontext1960():
    a = np.random.ranf(10 * (1960 + 9)).reshape(10, -1)
    for p in [pcl.PointCloud.UniqueShapeContext1960(a), pcl.PointCloud.UniqueShapeContext1960.from_array(a)]:
        assert np.allclose(p.descriptor, a[:, :1960])
        assert np.allclose(p.rf, a[:, 1960:])
        assert pcl.point_types.UniqueShapeContext1960.descriptorSize() == 1960


def test_buffers_shot352():
    a = np.random.ranf(10 * (352 + 9)).reshape(10, -1)
    for p in [pcl.PointCloud.SHOT352(a), pcl.PointCloud.SHOT352.from_array(a)]:
        assert np.allclose(p.descriptor, a[:, :352])
        assert np.allclose(p.rf, a[:, 352:])
        assert pcl.point_types.SHOT352.descriptorSize() == 352


def test_buffers_shot1344():
    a = np.random.ranf(10 * (1344 + 9)).reshape(10, -1)
    for p in [pcl.PointCloud.SHOT1344(a), pcl.PointCloud.SHOT1344.from_array(a)]:
        assert np.allclose(p.descriptor, a[:, :1344])
        assert np.allclose(p.rf, a[:, 1344:])
        assert pcl.point_types.SHOT1344.descriptorSize() == 1344


def test_buffers_uv():
    a = np.random.ranf(20).reshape(-1, 2)
    for p in [pcl.PointCloud.PointUV(a), pcl.PointCloud.PointUV.from_array(a)]:
        assert np.allclose(p.u, a[:, 0])
        assert np.allclose(p.v, a[:, 1])


def test_buffers_referenceframe():
    a = np.random.ranf(10 * 9).reshape(10, -1)
    for p in [pcl.PointCloud.ReferenceFrame(a), pcl.PointCloud.ReferenceFrame.from_array(a)]:
        assert np.allclose(p.rf, a)
        assert np.allclose(p.x_axis, a[:, :3])
        assert np.allclose(p.y_axis, a[:, 3:6])
        assert np.allclose(p.z_axis, a[:, 6:])


def test_buffers_pointdem():
    a = np.random.ranf(60).reshape(-1, 6)
    for p in [pcl.PointCloud.PointDEM(a), pcl.PointCloud.PointDEM.from_array(a)]:
        assert np.allclose(p.x, a[:, 0])
        assert np.allclose(p.y, a[:, 1])
        assert np.allclose(p.z, a[:, 2])
        assert np.allclose(p.xyz, a[:, :3])
        assert np.allclose(p.intensity, a[:, 3])
        assert np.allclose(p.intensity_variance, a[:, 4])
        assert np.allclose(p.height_variance, a[:, 5])
