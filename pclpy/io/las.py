import numpy as np
import laspy

from pclpy import pcl


def get_offset(path):
    with laspy.file.File(path) as f:
        return f.header.offset


def read(path, point_type, xyz_offset=None):
    assert point_type in "PointXYZ PointXYZI PointXYZINormal PointNormal PointXYZRGBNormal PointXYZRGBA"
    if xyz_offset is None:
        xyz_offset = np.array([0, 0, 0])
    with laspy.file.File(path) as f:
        supported_attrs = "x y z intensity normal_x normal_y normal_z curvature".split()  # rgb below
        point_type_attrs = [a for a in dir(getattr(pcl.point_types, point_type)) if not a.startswith("_")]
        pcl_attrs = [attr for attr in supported_attrs if attr in point_type_attrs]
        xyz_data = np.zeros((f.header.count, len(pcl_attrs)), "f")
        for n, attr in enumerate(pcl_attrs):
            val = getattr(f, attr)
            pos = "xyz".find(attr)
            if pos != -1:
                val -= xyz_offset[pos]
            xyz_data[:, n] = val
        data = [xyz_data]

        if all(c in point_type_attrs for c in "rgb") or "rgba" in point_type_attrs:
            data.append((np.array([f.red, f.green, f.blue]) / 2 ** 8).astype("u1").T)

        pc = getattr(pcl.PointCloud, point_type).from_array(*data)
    return pc


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


def write(cloud, path, write_extra_dimensions=True, scale=0.0001, xyz_offset=None):
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

        if xyz_offset is not None:
            f.header.offset = xyz_offset
            f.x = cloud.x.astype("d") + xyz_offset[0]
            f.y = cloud.y.astype("d") + xyz_offset[1]
            f.z = cloud.z.astype("d") + xyz_offset[2]
        else:
            f.header.offset = cloud.xyz.min(axis=0)
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
