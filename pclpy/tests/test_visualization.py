import os
import numpy as np
import pytest

import pclpy
from pclpy import pcl


def test_data(*args):
    return os.path.join("test_data", *args)


def test_cloud_viewer():
    pc = pclpy.io.read_las(test_data("street.las"))
    viewer = pcl.visualization.CloudViewer("viewer")
    viewer.show_cloud(pc, "hi")
    # while not viewer.was_stopped(1):
    #     pass
