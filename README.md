# pclpy

Automatically generated python bindings for the Point Cloud Library (PCL)
using pybind11 and CppHeaderParser.

__Current focus is on Windows x64 and python 3.5+__


## Why
While Cython is great, binding templated code isn't one of its strenghts.
The python-pcl are hard to maintain and adding features or point types is quite tedious.
With pybind11, we can use c++ templates directly.

## Features
- All point types are planned to be wrapped
- You can access point cloud attributes as numpy arrays using `point_cloud.x` or `point_cloud.xyz`

## Modules
- Build, but largely untested
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
- Should also work soon
    - ml
    - people
    - outofcore registration
    - visualization

## Not Implemented (WIP)
- Specialized templated methods
- Specialized templated classes
- Class operators
- PCL Exceptions
- Functions taking a function pointer or a boost::function as argument
- Others: see section "what to skip" in constants.py

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
- Write tests at the python level (there are no tests for the pybind11
  code generation and I think this would be too tedious. I plan to test at a higher level.)
- CI on Appveyor
- Make it pip installable as a wheel
- Make it installable on Linux and Mac
