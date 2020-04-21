# pclpy: PCL for python


[![conda](https://img.shields.io/conda/v/davidcaron/pclpy?label=conda)](https://anaconda.org/davidcaron/pclpy)
[![Python version](https://img.shields.io/badge/python-3.6%20%7C%203.7%20%7C%203.8-blue)](https://anaconda.org/davidcaron/pclpy)
[![conda](https://img.shields.io/conda/pn/davidcaron/pclpy?color=orange)](https://anaconda.org/davidcaron/pclpy)
[![PCL version](https://img.shields.io/badge/PCL-1.9.1-blue)](https://anaconda.org/conda-forge/pcl)

![.github/workflows/ci.yml](https://github.com/davidcaron/pclpy/workflows/.github/workflows/ci.yml/badge.svg)

Python bindings for the Point Cloud Library (PCL).
Generated from headers using CppHeaderParser and pybind11.

Install using conda: `conda install -c conda-forge -c davidcaron pclpy` (see _Installation_ below)

Contributions, issues, comments are welcome!

Github repository: https://www.github.com/davidcaron/pclpy

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

## Installation

We use conda to release pclpy. To install, use this command:

`conda install -c conda-forge -c davidcaron pclpy`

Don't forget to add both channels, or else conda won't be able to find all dependencies.

**Windows**: python **3.6** and **3.7** are supported

**Linux**: python **3.6**, **3.7** and **3.8** are supported


## Features

- Most point types are implemented (those specified by `PCL_ONLY_CORE_POINT_TYPES` in PCL)
- You can get a numpy view of point cloud data using python properties (e.g. `cloud.x` or `cloud.xyz`)
- boost::shared_ptr is handled by pybind11 so it's completely abstracted at the python level
- laspy integration for reading/writing las files

## Example

You can use either a high level, more pythonic api, or the wrapper over the PCL api.
The wrapper is meant to be as close as possible to the original PCL C++ api.

Here is how you would use the library to process Moving Least Squares.
See the PCL documentation: http://pointclouds.org/documentation/tutorials/resampling.php

Using the higher level api:

```python
import pclpy

# read a las file
point_cloud = pclpy.read("street.las", "PointXYZRGBA")
# compute mls
output = point_cloud.moving_least_squares(search_radius=0.05, compute_normals=True, num_threads=8)
```

Or the wrapper over the PCL api:

```python
import pclpy
from pclpy import pcl

point_cloud = pclpy.read("street.las", "PointXYZRGBA")
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

You can see the wrapper is very close to the C++ version:

``` c++
// C++ version

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::io::loadPCDFile ("bunny.pcd", *point_cloud);
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
- io (meshes are not implemented)
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
#### These modules are skipped for now
- ml
- people
- outofcore
- registration
- visualization
- every module not in the PCL Windows release (gpu, cuda, etc.)

## Not Implemented
(see [github issues](https://github.com/davidcaron/pclpy/issues)
and the _what to skip_ section in `generators/config.py`)

## Building

Build scripts are in the `scripts` folder.

1. Create your conda environment:
    `conda env create -n pclpy -f environment.yml`

2. Activate your environment:
    `conda activate pclpy`

3. Install development dependencies:
    `pip install -r requirements-dev.txt`

4. Download a copy of PCL
    Windows: `powershell scripts\download_pcl.ps1`
    Linux: `scripts\download_pcl.sh`

5. Generate pybind11 bindings
    Windows: `powershell scripts\generate_points_and_bindings.ps1`
    Linux: `scripts\generate_points_and_bindings.sh`

6. For development, build inplace using python
    `python setup.py build_ext -i`
    
    For a release, use the scripts/conda_build.bat (or conda_build.sh) script

On Windows, these setup.py arguments can cpeed up the build:
    - --msvc-mp-build enables a multiprocessed build
    - --msvc-no-code-link makes the linking step faster (not meant for releases)
    - --use-clcache to cache msvc builds (clcache must be installed)

## Roadmap
- Wrap as much of PCL as reasonably possible
- More tests
