
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

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingApproxNearestPairPointCloudCoherence(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::ApproxNearestPairPointCloudCoherence<PointInT>;
    using PointCoherencePtr = Class::PointCoherencePtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, pcl::tracking::NearestPairPointCloudCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineTrackingApproxNearestPairPointCloudCoherenceFunctions(py::module &m) {
}

void defineTrackingApproxNearestPairPointCloudCoherenceClasses(py::module &sub_module) {
    py::module sub_module_ApproxNearestPairPointCloudCoherence = sub_module.def_submodule("ApproxNearestPairPointCloudCoherence", "Submodule ApproxNearestPairPointCloudCoherence");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::InterestPoint>(sub_module_ApproxNearestPairPointCloudCoherence, "InterestPoint");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointDEM>(sub_module_ApproxNearestPairPointCloudCoherence, "PointDEM");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointNormal>(sub_module_ApproxNearestPairPointCloudCoherence, "PointNormal");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointSurfel>(sub_module_ApproxNearestPairPointCloudCoherence, "PointSurfel");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointWithRange>(sub_module_ApproxNearestPairPointCloudCoherence, "PointWithRange");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointWithScale>(sub_module_ApproxNearestPairPointCloudCoherence, "PointWithScale");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointWithViewpoint>(sub_module_ApproxNearestPairPointCloudCoherence, "PointWithViewpoint");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZ>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZ");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZHSV>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZHSV");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZI>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZI");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZINormal>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZINormal");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZL>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZL");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZLNormal>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZLNormal");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZRGB>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZRGB");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZRGBA");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBL>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZRGBL");
    defineTrackingApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBNormal>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZRGBNormal");
}