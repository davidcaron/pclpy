import pytest
import os
import numpy as np
import laspy

from pclpy import pcl
import pclpy


def test_data(*args):
    return os.path.join("test_data", *args)


def test_extract_indices_simple():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    extract = pcl.filters.ExtractIndices.PointXYZ()
    extract.setInputCloud(pc)
    pi = pcl.PointIndices()
    pi.indices.append(3)
    extract.setIndices(pi)
    out = pcl.PointCloud.PointXYZ()
    extract.filter(out)
    assert np.allclose(out.xyz, np.array([[12.259, -0.124, 21.9086]], "f"))


def test_radius_outlier_removal_simple():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output = pclpy.radius_outlier_removal(pc, search_radius=0.5, min_neighbors=10)
    assert output.size() == 4304


def test_radius_outlier_removal_indices():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    indices = pcl.vectors.Int(np.arange(10, 1000, dtype="i"))
    output = pclpy.radius_outlier_removal(pc,
                                          search_radius=0.5,
                                          min_neighbors=10,
                                          indices=indices)
    assert output.size() == 889


def test_radius_outlier_removal_negative():
    pc = pclpy.read(test_data("street_thinned.las"), "PointXYZ")
    output = pclpy.radius_outlier_removal(pc,
                                          search_radius=0.5,
                                          min_neighbors=10,
                                          negative=True)
    assert output.size() == 721
