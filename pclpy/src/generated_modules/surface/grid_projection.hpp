
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/surface/grid_projection.h>



template <typename PointNT>
void defineSurfaceGridProjection(py::module &m, std::string const & suffix) {
    using Class = pcl::GridProjection<PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using HashMap = Class::HashMap;
    py::class_<Class, pcl::SurfaceReconstruction<PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setResolution", &Class::setResolution, "resolution"_a);
    cls.def("setPaddingSize", &Class::setPaddingSize, "padding_size"_a);
    cls.def("setNearestNeighborNum", &Class::setNearestNeighborNum, "k"_a);
    cls.def("setMaxBinarySearchLevel", &Class::setMaxBinarySearchLevel, "max_binary_search_level"_a);
    cls.def("getResolution", &Class::getResolution);
    cls.def("getPaddingSize", &Class::getPaddingSize);
    cls.def("getNearestNeighborNum", &Class::getNearestNeighborNum);
    cls.def("getMaxBinarySearchLevel", &Class::getMaxBinarySearchLevel);
    cls.def("getCellHashMap", &Class::getCellHashMap);
    cls.def("getVectorAtDataPoint", &Class::getVectorAtDataPoint);
    cls.def("getSurface", &Class::getSurface);
        
}

void defineSurfaceGridProjectionFunctions(py::module &m) {
}

void defineSurfaceGridProjectionClasses(py::module &sub_module) {
}