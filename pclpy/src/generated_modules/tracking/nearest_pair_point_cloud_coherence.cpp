
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

#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingNearestPairPointCloudCoherence(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::NearestPairPointCloudCoherence<PointInT>;
    using PointCoherencePtr = Class::PointCoherencePtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using BaseClass = Class::BaseClass;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using SearchPtr = Class::SearchPtr;
    using SearchConstPtr = Class::SearchConstPtr;
    py::class_<Class, pcl::tracking::PointCloudCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("setSearchMethod", &Class::setSearchMethod, "search"_a);
    cls.def("setTargetCloud", &Class::setTargetCloud, "cloud"_a);
    cls.def("setMaximumDistance", &Class::setMaximumDistance, "val"_a);
    cls.def("getSearchMethod", &Class::getSearchMethod);
        
}

void defineTrackingNearestPairPointCloudCoherenceFunctions(py::module &m) {
}

void defineTrackingNearestPairPointCloudCoherenceClasses(py::module &sub_module) {
    py::module sub_module_NearestPairPointCloudCoherence = sub_module.def_submodule("NearestPairPointCloudCoherence", "Submodule NearestPairPointCloudCoherence");
    defineTrackingNearestPairPointCloudCoherence<pcl::InterestPoint>(sub_module_NearestPairPointCloudCoherence, "InterestPoint");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointDEM>(sub_module_NearestPairPointCloudCoherence, "PointDEM");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointNormal>(sub_module_NearestPairPointCloudCoherence, "PointNormal");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointSurfel>(sub_module_NearestPairPointCloudCoherence, "PointSurfel");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointWithRange>(sub_module_NearestPairPointCloudCoherence, "PointWithRange");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointWithScale>(sub_module_NearestPairPointCloudCoherence, "PointWithScale");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointWithViewpoint>(sub_module_NearestPairPointCloudCoherence, "PointWithViewpoint");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZ>(sub_module_NearestPairPointCloudCoherence, "PointXYZ");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZHSV>(sub_module_NearestPairPointCloudCoherence, "PointXYZHSV");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZI>(sub_module_NearestPairPointCloudCoherence, "PointXYZI");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZINormal>(sub_module_NearestPairPointCloudCoherence, "PointXYZINormal");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZL>(sub_module_NearestPairPointCloudCoherence, "PointXYZL");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZLNormal>(sub_module_NearestPairPointCloudCoherence, "PointXYZLNormal");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZRGB>(sub_module_NearestPairPointCloudCoherence, "PointXYZRGB");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZRGBA>(sub_module_NearestPairPointCloudCoherence, "PointXYZRGBA");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZRGBL>(sub_module_NearestPairPointCloudCoherence, "PointXYZRGBL");
    defineTrackingNearestPairPointCloudCoherence<pcl::PointXYZRGBNormal>(sub_module_NearestPairPointCloudCoherence, "PointXYZRGBNormal");
}