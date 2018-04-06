
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/keypoints/keypoint.h>



template <typename PointInT, typename PointOutT>
void defineKeypointsKeypoint(py::module &m, std::string const & suffix) {
    using Class = Keypoint<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using BaseClass = Class::BaseClass;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("search_surface", &Class::getSearchSurface, &Class::setSearchSurface);
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def_property("k_search", &Class::getKSearch, &Class::setKSearch);
    cls.def_property("radius_search", &Class::getRadiusSearch, &Class::setRadiusSearch);
    cls.def("compute", &Class::compute);
    cls.def("search_for_neighbors", &Class::searchForNeighbors);
        
}

void defineKeypointsKeypointClasses(py::module &sub_module) {
}