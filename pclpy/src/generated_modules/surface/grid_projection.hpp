
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/grid_projection.h>



template <typename PointNT>
void defineSurfaceGridProjection(py::module &m, std::string const & suffix) {
    using Class = GridProjection<PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using HashMap = Class::HashMap;
    py::class_<Class, SurfaceReconstruction<PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("resolution", &Class::getResolution, &Class::setResolution);
    cls.def_property("padding_size", &Class::getPaddingSize, &Class::setPaddingSize);
    cls.def_property("nearest_neighbor_num", &Class::getNearestNeighborNum, &Class::setNearestNeighborNum);
    cls.def_property("max_binary_search_level", &Class::getMaxBinarySearchLevel, &Class::setMaxBinarySearchLevel);
        
}

void defineSurfaceGridProjectionClasses(py::module &sub_module) {
}