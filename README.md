# pclpy

Automatically generated python bindings for the Point Cloud Library (PCL)
using pybind11 and CppHeaderParser.

__The library is in active development, do not
use in production and the api is likely to change.__

Only Windows x64 and python 3.5+ are supported at the moment.


## Why
While Cython is great, binding templated code isn't one of its strenghts.
The python-pcl bindings contain a lot of repeated code and adding
features or point types is quite tedious.
Also, python-pcl implement only a subset of PCL's classes and point types.

With pybind11, we can use c++ templates directly.
The goal is to bind as much of the library as possible.

## Features
- All point types are planned to be wrapped
- You can access point cloud attributes as numpy arrays using `point_cloud.x` or `point_cloud.xyz`

## Example

Here is how you would use the library to process a Moving Least Squares.

(see PCL documentation: http://pointclouds.org/documentation/tutorials/resampling.php)

```python
import pclpy
from pclpy import pcl

point_cloud = pclpy.io.read_las(test_data("street.las"))
mls = pcl.surface.MovingLeastSquaresOMP.PointXYZRGBA_PointNormal()
mls.search_radius = 0.05
mls.polynomial_fit = False
mls.set_number_of_threads(12)
mls.input_cloud = input
tree = pcl.search.KdTree.PointXYZRGBA()
mls.search_method = tree
mls.set_compute_normals(True)
output = pcl.PointCloudPointNormal()
mls.process(output)
```

## Modules
- These modules build. They should work, but are largely untested.
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
- Should also work soon
    - ml
    - people
    - outofcore registration

## Not Implemented (WIP)
- Specialized templated classes
- Class operators
- PCL Exceptions
- Functions taking a function pointer or a boost::function as argument
- Others: see section "what to skip" in constants.py and todo.md

## To build
- Download PCL release for Windows (PCL-1.8.1-AllInOne-msvc2015-win64.exe) at:
    https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2015-win64.exe
- Generated modules are in the _src_ folder
- Must be built with x64 version of cl.exe (see workaround in setup.py
- Useful setup.py arguments:
    - --msvc-mp-build should enable a multiprocessed build
    - --msvc-no-code-link makes linking much faster (do not use for releases, see setup.py description)
    - --use-clcache to cache msvc builds using clcache (must be installed)
- PCL_ROOT environment variable must be set to the installation directory of PCL
- The goal is to make the binding pip installable once PCL is installed

Note: missing file from windows build : 2d/impl/kernel.hpp

## Roadmap
- Wrap as much of PCL as reasonably possible
- More tests
- CI on Appveyor
- Make it pip installable as a wheel
- Make it installable on Linux and Mac
