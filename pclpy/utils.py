from . import pcl
from copy import copy
from functools import update_wrapper

point_cloud_types = [t for t in dir(pcl.PointCloud) if not t.startswith("__")]


def get_point_cloud_type(*point_clouds):
    return "_".join([type(pc).__name__ for pc in point_clouds])


def register_point_cloud_function(func):
    function_name = func.__name__
    for point_cloud_type in point_cloud_types:
        pc_type = getattr(pcl.PointCloud, point_cloud_type)
        setattr(pc_type, function_name, func)
    return func


def register_alias(function_name, func):
    f = update_wrapper(copy(func), func)
    f.__name__ = function_name
    register_point_cloud_function(f)
    return f
