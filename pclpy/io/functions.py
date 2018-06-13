import os

from . import las


def read(path: str, point_type: str, xyz_offset=None):
    """
    Read a point cloud by guessing the filetype from its extension.
    """
    extension = os.path.splitext(path)[1][1:].lower()
    if extension == "las":
        return las.read(path=path, point_type=point_type, xyz_offset=xyz_offset)

    raise ValueError("Can't guess filetype from extension ('%s')" % extension)

