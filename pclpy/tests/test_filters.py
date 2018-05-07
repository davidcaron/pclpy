import pytest
import os
import numpy as np
import laspy

from pclpy import pcl
import pclpy


def test_data(*args):
    return os.path.join("test_data", *args)


def test_extract_indices_simple():
    pc = pclpy.io.las.read(test_data("street_thinned.las"), "PointXYZ")
    extract = pcl.filters.ExtractIndices.PointXYZ()
    extract.setInputCloud(pc)
    pi = pcl.PointIndices()
    pi.indices.append(3)
    extract.setIndices(pi)
    out = pcl.PointCloud.PointXYZ()
    extract.filter(out)
    assert np.allclose(out.xyz, np.array([[12.259, -0.124, 21.9086]], "f"))
