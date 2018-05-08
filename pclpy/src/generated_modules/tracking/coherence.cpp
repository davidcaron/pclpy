
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

#include <pcl/tracking/coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingPointCloudCoherence(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::PointCloudCoherence<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCoherencePtr = Class::PointCoherencePtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("compute", &Class::compute, "cloud"_a, "indices"_a, "w_i"_a);
    cls.def("initCompute", &Class::initCompute);
    cls.def("addPointCoherence", &Class::addPointCoherence, "coherence"_a);
    cls.def("setPointCoherences", &Class::setPointCoherences, "coherences"_a);
    cls.def("setTargetCloud", &Class::setTargetCloud, "cloud"_a);
    cls.def("getPointCoherences", &Class::getPointCoherences);
        
}

template <typename PointInT>
void defineTrackingPointCoherence(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::PointCoherence<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("compute", &Class::compute, "source"_a, "target"_a);
        
}

void defineTrackingCoherenceFunctions(py::module &m) {
}

void defineTrackingCoherenceClasses(py::module &sub_module) {
    py::module sub_module_PointCloudCoherence = sub_module.def_submodule("PointCloudCoherence", "Submodule PointCloudCoherence");
    defineTrackingPointCloudCoherence<pcl::InterestPoint>(sub_module_PointCloudCoherence, "InterestPoint");
    defineTrackingPointCloudCoherence<pcl::PointDEM>(sub_module_PointCloudCoherence, "PointDEM");
    defineTrackingPointCloudCoherence<pcl::PointNormal>(sub_module_PointCloudCoherence, "PointNormal");
    defineTrackingPointCloudCoherence<pcl::PointSurfel>(sub_module_PointCloudCoherence, "PointSurfel");
    defineTrackingPointCloudCoherence<pcl::PointWithRange>(sub_module_PointCloudCoherence, "PointWithRange");
    defineTrackingPointCloudCoherence<pcl::PointWithScale>(sub_module_PointCloudCoherence, "PointWithScale");
    defineTrackingPointCloudCoherence<pcl::PointWithViewpoint>(sub_module_PointCloudCoherence, "PointWithViewpoint");
    defineTrackingPointCloudCoherence<pcl::PointXYZ>(sub_module_PointCloudCoherence, "PointXYZ");
    defineTrackingPointCloudCoherence<pcl::PointXYZHSV>(sub_module_PointCloudCoherence, "PointXYZHSV");
    defineTrackingPointCloudCoherence<pcl::PointXYZI>(sub_module_PointCloudCoherence, "PointXYZI");
    defineTrackingPointCloudCoherence<pcl::PointXYZINormal>(sub_module_PointCloudCoherence, "PointXYZINormal");
    defineTrackingPointCloudCoherence<pcl::PointXYZL>(sub_module_PointCloudCoherence, "PointXYZL");
    defineTrackingPointCloudCoherence<pcl::PointXYZLNormal>(sub_module_PointCloudCoherence, "PointXYZLNormal");
    defineTrackingPointCloudCoherence<pcl::PointXYZRGB>(sub_module_PointCloudCoherence, "PointXYZRGB");
    defineTrackingPointCloudCoherence<pcl::PointXYZRGBA>(sub_module_PointCloudCoherence, "PointXYZRGBA");
    defineTrackingPointCloudCoherence<pcl::PointXYZRGBL>(sub_module_PointCloudCoherence, "PointXYZRGBL");
    defineTrackingPointCloudCoherence<pcl::PointXYZRGBNormal>(sub_module_PointCloudCoherence, "PointXYZRGBNormal");
    py::module sub_module_PointCoherence = sub_module.def_submodule("PointCoherence", "Submodule PointCoherence");
    defineTrackingPointCoherence<pcl::InterestPoint>(sub_module_PointCoherence, "InterestPoint");
    defineTrackingPointCoherence<pcl::Normal>(sub_module_PointCoherence, "Normal");
    defineTrackingPointCoherence<pcl::PointDEM>(sub_module_PointCoherence, "PointDEM");
    defineTrackingPointCoherence<pcl::PointNormal>(sub_module_PointCoherence, "PointNormal");
    defineTrackingPointCoherence<pcl::PointSurfel>(sub_module_PointCoherence, "PointSurfel");
    defineTrackingPointCoherence<pcl::PointWithRange>(sub_module_PointCoherence, "PointWithRange");
    defineTrackingPointCoherence<pcl::PointWithScale>(sub_module_PointCoherence, "PointWithScale");
    defineTrackingPointCoherence<pcl::PointWithViewpoint>(sub_module_PointCoherence, "PointWithViewpoint");
    defineTrackingPointCoherence<pcl::PointXYZ>(sub_module_PointCoherence, "PointXYZ");
    defineTrackingPointCoherence<pcl::PointXYZHSV>(sub_module_PointCoherence, "PointXYZHSV");
    defineTrackingPointCoherence<pcl::PointXYZI>(sub_module_PointCoherence, "PointXYZI");
    defineTrackingPointCoherence<pcl::PointXYZINormal>(sub_module_PointCoherence, "PointXYZINormal");
    defineTrackingPointCoherence<pcl::PointXYZL>(sub_module_PointCoherence, "PointXYZL");
    defineTrackingPointCoherence<pcl::PointXYZLNormal>(sub_module_PointCoherence, "PointXYZLNormal");
    defineTrackingPointCoherence<pcl::PointXYZRGB>(sub_module_PointCoherence, "PointXYZRGB");
    defineTrackingPointCoherence<pcl::PointXYZRGBA>(sub_module_PointCoherence, "PointXYZRGBA");
    defineTrackingPointCoherence<pcl::PointXYZRGBL>(sub_module_PointCoherence, "PointXYZRGBL");
    defineTrackingPointCoherence<pcl::PointXYZRGBNormal>(sub_module_PointCoherence, "PointXYZRGBNormal");
}