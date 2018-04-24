import numpy as np
import laspy

from pclpy import pcl


def get_offset(path):
    with laspy.file.File(path) as f:
        return f.header.offset


def read_las(path, colors=True, normals=False, offset=None):
    if offset is None:
        offset = np.array([0, 0, 0])
    with laspy.file.File(path) as f:
        f_dims = get_all_las_dims(f)
        has = lambda x: x in f_dims
        get_rgb = lambda: (np.array([f.red, f.green, f.blue]) / 2 ** 8).astype("u1").T
        get_xyz = lambda: np.array([f.x - offset[0], f.y - offset[1], f.z - offset[2]], "f").T
        get_xyznormals = lambda: np.array([f.x - offset[0], f.y - offset[1], f.z - offset[2],
                                           f.normal_x, f.normal_y, f.normal_z, f.curvature], "f").T
        colors = colors and has("red") and has("green") and has("blue")
        normals = normals and has("normal_x") and has("normal_y") and has("normal_z") and has("curvature")
        if colors and normals:
            rgb = get_rgb()
            xyznormals = get_xyznormals()
            p = pcl.PointCloud.PointXYZRGBNormal.from_array(xyznormals, rgb)
        elif colors:
            rgb = get_rgb()
            xyz = get_xyz()
            p = pcl.PointCloud.PointXYZRGBA.from_array(xyz, rgb)
        elif normals:
            xyznormals = get_xyznormals()
            p = pcl.PointCloud.PointNormal.from_array(xyznormals)
        else:
            xyz = get_xyz()
            p = pcl.PointCloud.PointXYZ.from_array(xyz)
    return p


def get_all_las_dims(las_file):
    return list(las_file.points.dtype[0].fields.keys())


def get_extra_dims(cloud):
    standard_dims = {"x", "y", "z", "intensity", "sensor_origin_", "sensor_orientation", "r", "g", "b"}
    extra_dims = []
    for attr in dir(cloud):
        if attr in standard_dims:
            continue
        a = getattr(cloud, attr)
        if isinstance(a, np.ndarray) and a.ndim == 1:
            extra_dims.append(attr)
    return sorted(extra_dims)


def to_las(cloud, path, write_extra_dimensions=True, scale=0.0001, offset=None):
    has = lambda x: hasattr(cloud, x)

    if not all([has("x") and has("y") and has("z")]):
        raise ValueError("Not a XYZ point type %s" % type(cloud))

    has_color = has("r") and has("g") and has("b")
    point_format = 0
    if has_color:
        point_format = 2
    header = laspy.header.Header(point_format=point_format)

    with laspy.file.File(path, mode="w", header=header) as f:
        extra_dims = []
        if write_extra_dimensions:
            extra_dims = get_extra_dims(cloud)
            for dim in extra_dims:
                f.define_new_dimension(dim, 5, dim)

        f.header.scale = (scale, scale, scale)

        if offset is not None:
            f.header.offset = offset
            f.x = cloud.x + offset[0]
            f.y = cloud.y + offset[1]
            f.z = cloud.z + offset[2]
        else:
            min_ = np.array([cloud.x.min(), cloud.y.min(), cloud.z.min()])
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

        for dim in extra_dims:
            setattr(f, dim, getattr(cloud, dim))
