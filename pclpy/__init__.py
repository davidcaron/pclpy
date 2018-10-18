import platform

if platform.system() == "Windows":
    import pclpy_dependencies

import pclpy.pcl as pcl

from pclpy.io.functions import read
from pclpy.io.las import read as read_las
from pclpy.io.las import write as write_las

from pclpy.view.vtk import Viewer

from pclpy.api import (extract_clusters,
                       compute_normals,
                       region_growing,
                       show,
                       moving_least_squares,
                       mls,
                       radius_outlier_removal,
                       ror,
                       octree_voxel_downsample,
                       fit,
                       )
