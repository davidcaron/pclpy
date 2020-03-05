import subprocess

PCL_VERSION = "1.9"

libs_to_build = ['common', 'features', 'filters', 'geometry', 'io', 'kdtree', 'keypoints', 'octree',
                 'recognition', 'sample_consensus', 'search', 'segmentation', 'stereo', 'surface',
                 'tracking', 'visualization']
pcl_libraries = ["pcl_%s-%s" % (lib, PCL_VERSION) for lib in libs_to_build]


def pkg_config_multi(arg, skip_chars=0):
    output = []
    for lib in pcl_libraries:
        for value in pkg_config(arg, lib):
            output.append(value[skip_chars:])
    return list(set(output))


def pkg_config(arg, lib):
    command = ["pkg-config", arg, lib]
    output = subprocess.check_output(command).decode().strip()
    return output.split()


def get_include_dir():
    for path in pkg_config_multi("--cflags-only-I", skip_chars=2):
        if path.endswith("pcl-%s" % PCL_VERSION):
            return path
