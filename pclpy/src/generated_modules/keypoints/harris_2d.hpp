
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/keypoints/harris_2d.h>



template <typename PointInT, typename PointOutT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineKeypointsHarrisKeypoint2D(py::module &m, std::string const & suffix) {
    using Class = HarrisKeypoint2D<PointInT, PointOutT, IntensityT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, Keypoint<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::ResponseMethod>(cls, "response_method")
        .value("HARRIS", Class::ResponseMethod::HARRIS)
        .value("NOBLE", Class::ResponseMethod::NOBLE)
        .value("LOWE", Class::ResponseMethod::LOWE)
        .value("TOMASI", Class::ResponseMethod::TOMASI)
        .export_values();
    cls.def(py::init<Class::ResponseMethod, int, int, int, float>(), "method"_a=Class::HARRIS, "window_width"_a=3, "window_height"_a=3, "min_distance"_a=5, "threshold"_a=0.0);
    cls.def("set_method", &Class::setMethod);
    cls.def("set_window_width", &Class::setWindowWidth);
    cls.def("set_window_height", &Class::setWindowHeight);
    cls.def("set_skipped_pixels", &Class::setSkippedPixels);
    cls.def("set_minimal_distance", &Class::setMinimalDistance);
    cls.def("set_threshold", &Class::setThreshold);
    cls.def("set_non_max_supression", &Class::setNonMaxSupression);
    cls.def("set_refine", &Class::setRefine);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineKeypointsHarris2dClasses(py::module &sub_module) {
}