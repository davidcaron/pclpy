
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/keypoints/sift_keypoint.h>



template <typename PointInT, typename PointOutT>
void defineKeypointsSIFTKeypoint(py::module &m, std::string const & suffix) {
    using Class = pcl::SIFTKeypoint<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    py::class_<Class, pcl::Keypoint<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setScales", &Class::setScales, "min_scale"_a, "nr_octaves"_a, "nr_scales_per_octave"_a);
    cls.def("setMinimumContrast", &Class::setMinimumContrast, "min_contrast"_a);
        
}

template<typename PointT>
void defineKeypointsSIFTKeypointFieldSelector(py::module &m, std::string const & suffix) {
    using Class = pcl::SIFTKeypointFieldSelector<PointT>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    // Operators not implemented (operator());
        
}

void defineKeypointsSiftKeypointFunctions(py::module &m) {
}

void defineKeypointsSiftKeypointClasses(py::module &sub_module) {
}