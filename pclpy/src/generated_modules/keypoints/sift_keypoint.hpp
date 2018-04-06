
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/keypoints/sift_keypoint.h>



template <typename PointInT, typename PointOutT>
void defineKeypointsSIFTKeypoint(py::module &m, std::string const & suffix) {
    using Class = SIFTKeypoint<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    py::class_<Class, Keypoint<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_scales", &Class::setScales);
    cls.def("set_minimum_contrast", &Class::setMinimumContrast);
        
}

template<typename PointT>
void defineKeypointsSIFTKeypointFieldSelector(py::module &m, std::string const & suffix) {
    using Class = SIFTKeypointFieldSelector<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    // Operators not implemented (operator());
        
}







void defineKeypointsSiftKeypointClasses(py::module &sub_module) {
}