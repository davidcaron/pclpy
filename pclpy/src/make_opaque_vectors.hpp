
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <Eigen/Geometry>

PYBIND11_MAKE_OPAQUE(std::vector<pcl::PointIndices>);
PYBIND11_MAKE_OPAQUE(std::vector<int>);
PYBIND11_MAKE_OPAQUE(std::vector<pcl::PointXYZRGBA, Eigen::aligned_allocator<pcl::PointXYZRGBA>>);
PYBIND11_MAKE_OPAQUE(Eigen::Quaternionf);


// todo: there are way more than this