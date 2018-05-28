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
    viewer = pcl.visualization.PCLVisualizer("viewer", False)
    viewer.setBackgroundColor(0, 0, 0)
    rgb = pcl.visualization.PointCloudColorHandlerRGBAField.PointXYZRGBA(pc)
    viewer.addPointCloud(pc, rgb, "sample cloud")
    viewer.setPointCloudRenderingProperties(0, 3, "sample cloud")
    viewer.addCoordinateSystem(1.0)
    viewer.initCameraParameters()
    viewer.resetCamera()

    # don't show the cloud in automated tests
    # while not viewer.wasStopped():
    #     viewer.spinOnce(10)


def test_viewer_intensity():
    pc = pclpy.io.read(test_data("street.las"), "PointXYZI")
    viewer = pclpy.view.vtk.Viewer(pc)
    # don't show the cloud in automated tests
    # viewer.show()


def test_callbacks():
    pc = pclpy.io.read(test_data("street.las"), "PointXYZRGBA")
    viewer = pcl.visualization.PCLVisualizer("viewer", True)
    viewer.addPointCloud(pc)
    viewer.resetCamera()

    def handle_event_key(event):
        assert isinstance(event, pcl.visualization.KeyboardEvent)

    def handle_event_mouse(event):
        assert isinstance(event, pcl.visualization.MouseEvent)

    def handle_event_point(event):
        assert isinstance(event, pcl.visualization.PointPickingEvent)
        index = event.getPointIndex()
        point = pc.at(index)
        print(point.r, point.g, point.b)

    def handle_event_area(event):
        # use x to toggle rectangle selection
        assert isinstance(event, pcl.visualization.AreaPickingEvent)
        indices = pcl.vectors.Int()
        event.getPointsIndices(indices)

    viewer.registerKeyboardCallback(handle_event_key)
    viewer.registerMouseCallback(handle_event_mouse)
    viewer.registerPointPickingCallback(handle_event_point)
    viewer.registerAreaPickingCallback(handle_event_area)
    # don't show the cloud in automated tests
    # while not viewer.wasStopped():
    #     viewer.spinOnce(50)


def test_open_in_new_window():
    pc = pclpy.io.read(test_data("street.las"), "PointXYZRGBA")
    viewer = pcl.visualization.PCLVisualizer("viewer", False)
    viewer.addPointCloud(pc)
    viewer.resetCamera()

    viewers = [viewer]

    def handle_event_area(event):
        # use x to toggle rectangle selection
        assert isinstance(event, pcl.visualization.AreaPickingEvent)
        indices = pcl.vectors.Int()
        event.getPointsIndices(indices)

        other_viewer = pcl.visualization.PCLVisualizer("viewer", False)
        rgb = np.array([pc.r[indices], pc.g[indices], pc.b[indices]]).T
        other_pc = pcl.PointCloud.PointXYZRGBA.from_array(pc.xyz[indices], rgb)
        other_viewer.addPointCloud(other_pc)
        other_viewer.resetCamera()
        viewers.append(other_viewer)

    viewer.registerAreaPickingCallback(handle_event_area)

    # don't show the cloud in automated tests
    # while not all(v.wasStopped() for v in viewers):
    #     for v in viewers:
    #         if not v.wasStopped():
    #             v.spinOnce(50)


def test_camera_parameters():
    viewer = pcl.visualization.PCLVisualizer("viewer", False)
    cameras = pcl.vectors.Camera()
    viewer.getCameras(cameras)
    cam = cameras[0]
    assert cam.pos()[1] != 2
    cam.pos()[1] = 2
    assert cam.pos()[1] == 2
