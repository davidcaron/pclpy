# pclpy: Point Cloud Library for python

Automatically generated python bindings for the Point Cloud Library (PCL)
using pybind11 and CppHeaderParser.

__The library is in active development, do not
use in production. The api is likely to change.__

Only Windows x64 and python 3.5+ are supported at the moment.


## Why ?
The _python-pcl_ bindings contain a lot of repeated code. Maintaining, adding
features or point types is quite tedious.
While Cython is great, binding templated code isn't one of its strenghts.
Also, python-pcl implement only a subset of PCL's classes and point types.

Using pybind11, we can use c++ templates directly.
The goal is to wrap as much of the library as possible.

## Features
- ~~All~~ About half of the point types are implemented (those specified by the default msvc compile flags)
- You can view point cloud data as numpy arrays using `cloud.x` or `cloud.xyz`
- boost::shared_ptr is handled by pybind11 so it's completely abstracted at the python level

## Example

Here is how you would use the library to process Moving Least Squares.
See the PCL documentation: http://pointclouds.org/documentation/tutorials/resampling.php

```python
import pclpy
from pclpy import pcl

point_cloud = pclpy.io.read_las(test_data("street.las"))
mls = pcl.surface.MovingLeastSquaresOMP.PointXYZRGBA_PointNormal()
tree = pcl.search.KdTree.PointXYZRGBA()
mls.setSearchRadius(0.05)
mls.setPolynomialFit(False)
mls.setNumberOfThreads(12)
mls.setInputCloud(point_cloud)
mls.setSearchMethod(tree)
mls.setComputeNormals(True)
output = pcl.PointCloud.PointNormal()
mls.process(output)
```

You can see it's quite similar to the C++ version:

``` c++
// C++ version

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::io::loadPCDFile ("bun0.pcd", *point_cloud);
pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> mls;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
mls.setSearchRadius (0.05);
mls.setPolynomialFit (false);
mls.setNumberOfThreads (12);
mls.setInputCloud (point_cloud);
mls.setSearchMethod (tree);
mls.setComputeNormals (true);
pcl::PointCloud<pcl::PointNormal> output;
mls.process (output);
```

## Modules
- These modules should work, but are untested
    - 2d
    - common
    - geometry
    - features
    - filters
    - io
    - kdtree
    - keypoints
    - octree
    - recognition
    - sample_consensus
    - search
    - segmentation
    - stereo
    - surface
    - tracking
    - visualization
- These modules are skipped for now
    - ml
    - people
    - outofcore
    - registration
    - every other module not built in the precompiled windows binaries

## Not Implemented
(see todo.md)

## To build
- Download PCL release for Windows (PCL-1.8.1-AllInOne-msvc2015-win64.exe) at:
    https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2015-win64.exe
- PCL_ROOT environment variable must be set to the installation directory of PCL
- About requirements:
    - Install pybind11 from github (2.3dev version) it includes a necessary bug fix
    - Install CppHeaderParser from https://github.com/davidcaron/CppHeaderParser (specific bug fixes)
- Generate modules using `generate_pybind11_bindings.py`
- There is a missing file from the PCL release that you should get from the github repo: 2d/impl/kernel.hpp
- Must be built with x64 version of cl.exe because of the large memory usage (see workaround in setup.py)
- python setup.py install
- Useful setup.py arguments:
    - --msvc-mp-build should enable a multiprocessed build
    - --msvc-no-code-link makes linking much faster (do not use for releases, see setup.py description)
    - --use-clcache to cache msvc builds using clcache (must be installed)
    - --debug to build in debug mode

## Roadmap
- Wrap as much of PCL as reasonably possible
- More tests
- CI on Appveyor
- Upload wheels to Pypi
- Make it installable on Linux and Mac
