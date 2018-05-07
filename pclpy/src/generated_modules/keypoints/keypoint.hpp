
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/keypoints/keypoint.h>



template <typename PointInT, typename PointOutT>
void defineKeypointsKeypoint(py::module &m, std::string const & suffix) {
    using Class = pcl::Keypoint<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using BaseClass = Class::BaseClass;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("compute", &Class::compute, "output"_a);
    cls.def("searchForNeighbors", &Class::searchForNeighbors, "index"_a, "parameter"_a, "indices"_a, "distances"_a);
    cls.def("setSearchSurface", &Class::setSearchSurface, "cloud"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("setKSearch", &Class::setKSearch, "k"_a);
    cls.def("setRadiusSearch", &Class::setRadiusSearch, "radius"_a);
    cls.def("getSearchSurface", &Class::getSearchSurface);
    cls.def("getSearchMethod", &Class::getSearchMethod);
    cls.def("getSearchParameter", &Class::getSearchParameter);
    cls.def("getKSearch", &Class::getKSearch);
    cls.def("getRadiusSearch", &Class::getRadiusSearch);
    cls.def("getKeypointsIndices", &Class::getKeypointsIndices);
        
}

void defineKeypointsKeypointFunctions(py::module &m) {
}

void defineKeypointsKeypointClasses(py::module &sub_module) {
}