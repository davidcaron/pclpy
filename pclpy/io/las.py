import numpy as np
import laspy

from pclpy import pcl


def read_las(path, read_colors=True):
    rgb = None
    with laspy.file.File(path) as f:
        xyz = np.array([f.x, f.y, f.z], "f").T
        if read_colors:
            rgb = np.array([f.red, f.green, f.blue])
    if read_colors:
        rgb = (rgb / 2 ** 8).astype("u1").T
        p = pcl.PointCloudXYZRGBA.from_array(xyz, rgb)
    else:
        p = pcl.PointCloudXYZ.from_array(xyz)
    return p


def to_las(cloud, path):
    has = lambda x: hasattr(cloud, x)

    if not all([has("x") and has("y") and has("z")]):
        raise ValueError("Not a XYZ point type %s" % type(cloud))

    has_color = has("r") and has("g") and has("b")
    point_format = 0
    if has_color:
        point_format = 2
    header = laspy.header.Header(point_format=point_format)

    with laspy.file.File(path, mode="w", header=header) as f:
        if has("labels"):
            f.define_new_dimension("labels", 5, "Labels")
        if has("normal_x"):
            f.define_new_dimension("normal_x", 5, "normal_x")
        if has("normal_y"):
            f.define_new_dimension("normal_y", 5, "normal_y")
        if has("normal_z"):
            f.define_new_dimension("normal_z", 5, "normal_z")
        if has("curvature"):
            f.define_new_dimension("curvature", 5, "curvature")
        # max_ = cloud.x.max(), cloud.y.max(), cloud.z.max()
        min_ = cloud.x.min(), cloud.y.min(), cloud.z.min()
        f.header.scale = (0.001, 0.001, 0.001)
        f.header.offset = min_
        f.x = cloud.x
        f.y = cloud.y
        f.z = cloud.z
        if has_color:
            f.red = cloud.r * 2 ** 8
            f.green = cloud.g * 2 ** 8
            f.blue = cloud.b * 2 ** 8
        if has("intensity"):
            f.intensity = cloud.intensity * 65535
        if has("labels"):
            f.labels = cloud.labels
        if has("normal_x"):
            f.normal_x = cloud.normal_x
        if has("normal_y"):
            f.normal_y = cloud.normal_y
        if has("normal_z"):
            f.normal_z = cloud.normal_z
        if has("curvature"):
            f.curvature = cloud.curvature * 1000
