# pclpy: PCL for python

[![PyPI](https://img.shields.io/pypi/v/pclpy.svg)](https://pypi.org/project/pclpy/)
[![PyPI Python version](https://img.shields.io/pypi/pyversions/pclpy.svg)](https://pypi.org/project/pclpy/)

Python bindings for the Point Cloud Library (PCL).
Generated from headers using CppHeaderParser and pybind11.

__This library is in active development, the api is likely to change.
The included modules do work, but tests are incomplete, and corner
cases are still common.__

Only Windows and python 3.6 x64 are supported at the moment.

Contributions, issues, comments are welcome!

Github repository: https://www.github.com/davidcaron/pclpy

Pypi: https://pypi.org/project/pclpy/

## Motivation
Many other python libraries tried to bind PCL.
The most popular one being python-pcl, which uses Cython.
While Cython is really powerful, binding C++ templates isn't one of
its strenghts (and PCL uses templates heavily).
The result for python-pcl is a lot of code repetition, which is hard
to maintain and to add features to, and incomplete bindings of PCL's classes
and point types.

Using pybind11, we use C++ directly. Templates, boost::smart_ptr and
the buffer protocol are examples of things that are simpler to implement.

The results so far are very promising. A large percentage of PCL is covered.

## Installing

#### Windows with python 3.6 x64
`pip install pclpy`

When pip installs the project, `pclpy_dependencies` is installed as a requirement.
This simple package contains only the PCL dlls required on Windows so you don't have
to download a PCL release or build it.

#### Linux

Not working for now. Contributions are welcome!

## Features
- All point types are implemented (those specified by the default msvc compile flags)
- You can view point cloud data as numpy arrays using `cloud.x` or `cloud.xyz`
- boost::shared_ptr is handled by pybind11 so it's completely abstracted at the python level
- laspy integration for reading/writing las files

## Example

Here is how you would use the library to process Moving Least Squares.
See the PCL documentation: http://pointclouds.org/documentation/tutorials/resampling.php

```python
import pclpy
from pclpy import pcl

point_cloud = pclpy.io.las.read(test_data("street.las"), "PointXYZRGBA")
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
#### These modules are skipped for now
- ml
- people
- outofcore
- registration
- every module not in the PCL Windows release (gpu, cuda, etc.)

## Not Implemented
(see [github issues](https://github.com/davidcaron/pclpy/issues)
and the _what to skip_ section in `generators/config.py`)

## To build
#### Windows with python 3.6 x64

- Download PCL release for Windows (PCL-1.8.1-AllInOne-msvc2017-win64.exe) at:
    https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2017-win64.exe
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
- Make it work on Linux
