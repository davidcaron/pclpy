
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/keypoints/harris_2d.h>



template <typename PointInT, typename PointOutT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
void defineKeypointsHarrisKeypoint2D(py::module &m, std::string const & suffix) {
    using Class = pcl::HarrisKeypoint2D<PointInT, PointOutT, IntensityT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudOut = Class::PointCloudOut;
    using KdTree = Class::KdTree;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, pcl::Keypoint<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::ResponseMethod>(cls, "ResponseMethod")
        .value("HARRIS", Class::ResponseMethod::HARRIS)
        .value("NOBLE", Class::ResponseMethod::NOBLE)
        .value("LOWE", Class::ResponseMethod::LOWE)
        .value("TOMASI", Class::ResponseMethod::TOMASI)
        .export_values();
    cls.def(py::init<Class::ResponseMethod, int, int, int, float>(), "method"_a=Class::HARRIS, "window_width"_a=3, "window_height"_a=3, "min_distance"_a=5, "threshold"_a=0.0);
    cls.def("setMethod", &Class::setMethod, "type"_a);
    cls.def("setWindowWidth", &Class::setWindowWidth, "window_width"_a);
    cls.def("setWindowHeight", &Class::setWindowHeight, "window_height"_a);
    cls.def("setSkippedPixels", &Class::setSkippedPixels, "skipped_pixels"_a);
    cls.def("setMinimalDistance", &Class::setMinimalDistance, "min_distance"_a);
    cls.def("setThreshold", &Class::setThreshold, "threshold"_a);
    cls.def("setNonMaxSupression", &Class::setNonMaxSupression, "bool"_a=false);
    cls.def("setRefine", &Class::setRefine, "do_refine"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineKeypointsHarris2dFunctions(py::module &m) {
}

void defineKeypointsHarris2dClasses(py::module &sub_module) {
}