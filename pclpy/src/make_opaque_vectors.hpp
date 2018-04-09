
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

PYBIND11_MAKE_OPAQUE(std::vector<pcl::PointIndices>);
PYBIND11_MAKE_OPAQUE(std::vector<int>);

// todo: there are way more than this