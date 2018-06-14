from . import pcl

point_cloud_types = [t for t in dir(pcl.PointCloud) if not t.startswith("__")]


def get_point_cloud_type(*point_clouds):
    return "_".join([type(pc).__name__ for pc in point_clouds])


def register_point_cloud_function(func):
    function_name = func.__name__
    for point_cloud_type in point_cloud_types:
        setattr(getattr(pcl.PointCloud, point_cloud_type), function_name, func)
    return func
