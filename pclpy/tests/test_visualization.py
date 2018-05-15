import os
import numpy as np
import pytest

import pclpy
from pclpy import pcl
import pclpy.view.vtk


def test_data(*args):
    return os.path.join("test_data", *args)


def test_cloud_viewer():
    pc = pclpy.io.read(test_data("street.las"), "PointXYZRGBA")
    viewer = pcl.visualization.CloudViewer("viewer")
    viewer.showCloud(pc, "hi")
    # don't show the cloud in automated tests
    # while not viewer.wasStopped(10):
    #     pass


def test_pcl_visualizer_simple():
    pc = pclpy.io.read(test_data("street.las"), "PointXYZRGBA")
    viewer = pcl.visualization.PCLVisualizer("viewer")
    viewer.setBackgroundColor(0, 0, 0)
    rgb = pcl.visualization.PointCloudColorHandlerRGBAField.PointXYZRGBA(pc)
    viewer.addPointCloud(pc, rgb, "sample cloud")
    viewer.setPointCloudRenderingProperties(0, 3, "sample cloud")
    viewer.addCoordinateSystem(1.0)
    viewer.initCameraParameters()
    # don't show the cloud in automated tests
    # while not viewer.wasStopped():
    #     viewer.spinOnce(10)


def test_viewer_intensity():
    pc = pclpy.io.read(test_data("street.las"), "PointXYZI")
    viewer = pclpy.view.vtk.Viewer(pc)
    # don't show the cloud in automated tests
    # viewer.show()
