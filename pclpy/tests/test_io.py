import os
import time

import pytest
import numpy as np
import laspy

from pclpy import pcl
import pclpy


def test_data(*args):
    return os.path.join("test_data", *args)


@pytest.fixture
def temp_las_path(request):
    path = test_data("temp.las")

    def fin():
        try:
            os.remove(path)

        except WindowsError:
            time.sleep(.05)
            os.remove(path)

    request.addfinalizer(fin)
    return path


def test_simple_io_pcd():
    pc = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read(test_data("bunny.pcd"), pc)
    assert pc.size() == 397


def test_las_xyz():
    pc = pclpy.io.las.read(test_data("street_thinned.las"), "PointXYZ")
    assert pc.size() == 5025


def test_las_rgb():
    pc = pclpy.io.las.read(test_data("street_thinned.las"), "PointXYZRGBA")
    assert pc.size() == 5025
    assert np.any(pc.r)


def test_las_intensity():
    pc = pclpy.io.las.read(test_data("street_thinned.las"), "PointXYZI")
    assert pc.size() == 5025
    assert np.any(pc.intensity)


def test_las_save_extra_dims(temp_las_path):
    a = np.random.ranf(70).reshape(-1, 7)
    pc = pcl.PointCloud.PointNormal.from_array(a)
    pclpy.io.las.write(pc, temp_las_path)

    assert os.path.exists(temp_las_path)
    with laspy.file.File(temp_las_path) as f:
        names = list(f.points.dtype[0].fields.keys())
        assert "normal_x" in names
        assert "normal_y" in names
        assert "normal_z" in names
        assert "curvature" in names


def test_las_read_normals():
    pc = pclpy.io.las.read(test_data("garbage_with_normals.las"), "PointNormal")
    assert hasattr(pc, "normal_x")
    assert hasattr(pc, "normal_y")
    assert hasattr(pc, "normal_z")
    assert hasattr(pc, "curvature")


def test_las_write_with_offset(temp_las_path):
    a = np.random.ranf(3).reshape(-1, 3)
    pc = pcl.PointCloud.PointXYZ.from_array(a)
    pclpy.io.las.write(pc, temp_las_path, xyz_offset=[10, 10, 10])
    pc = pclpy.io.las.read(temp_las_path, "PointXYZ")
    assert np.all(pc.x >= 10)
    assert np.all(pc.x <= 11)
    assert np.all(pc.y >= 10)
    assert np.all(pc.y <= 11)
    assert np.all(pc.z >= 10)
    assert np.all(pc.z <= 11)


def test_las_read_with_offset(temp_las_path):
    a = np.random.ranf(3).reshape(-1, 3)
    pc = pcl.PointCloud.PointXYZ.from_array(a)
    offset = [10, 10, 10]
    pclpy.io.las.write(pc, temp_las_path, xyz_offset=offset)
    file_offset = pclpy.io.las.get_offset(temp_las_path)
    assert np.allclose(offset, file_offset)
    pc = pclpy.io.las.read(temp_las_path, "PointXYZ", xyz_offset=offset)
    assert np.all(pc.x >= 0)
    assert np.all(pc.x <= 1)
    assert np.all(pc.y >= 0)
    assert np.all(pc.y <= 1)
    assert np.all(pc.z >= 0)
    assert np.all(pc.z <= 1)
