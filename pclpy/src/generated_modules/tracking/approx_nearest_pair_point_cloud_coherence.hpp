
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingApproxNearestPairPointCloudCoherence(py::module &m, std::string const & suffix) {
    using Class = tracking::ApproxNearestPairPointCloudCoherence<PointInT>;
    using PointCoherencePtr = Class::PointCoherencePtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, NearestPairPointCloudCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineTrackingApproxNearestPairPointCloudCoherenceClasses(py::module &sub_module) {
    py::module sub_module_ApproxNearestPairPointCloudCoherence = sub_module.def_submodule("ApproxNearestPairPointCloudCoherence", "Submodule ApproxNearestPairPointCloudCoherence");
    defineTrackingApproxNearestPairPointCloudCoherence<InterestPoint>(sub_module_ApproxNearestPairPointCloudCoherence, "InterestPoint");
    defineTrackingApproxNearestPairPointCloudCoherence<PointDEM>(sub_module_ApproxNearestPairPointCloudCoherence, "PointDEM");
    defineTrackingApproxNearestPairPointCloudCoherence<PointNormal>(sub_module_ApproxNearestPairPointCloudCoherence, "PointNormal");
    defineTrackingApproxNearestPairPointCloudCoherence<PointSurfel>(sub_module_ApproxNearestPairPointCloudCoherence, "PointSurfel");
    defineTrackingApproxNearestPairPointCloudCoherence<PointWithRange>(sub_module_ApproxNearestPairPointCloudCoherence, "PointWithRange");
    defineTrackingApproxNearestPairPointCloudCoherence<PointWithScale>(sub_module_ApproxNearestPairPointCloudCoherence, "PointWithScale");
    defineTrackingApproxNearestPairPointCloudCoherence<PointWithViewpoint>(sub_module_ApproxNearestPairPointCloudCoherence, "PointWithViewpoint");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZ>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZ");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZHSV>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZHSV");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZI>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZI");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZINormal>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZINormal");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZL>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZL");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZLNormal>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZLNormal");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZRGB>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZRGB");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZRGBA>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZRGBA");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZRGBL>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZRGBL");
    defineTrackingApproxNearestPairPointCloudCoherence<PointXYZRGBNormal>(sub_module_ApproxNearestPairPointCloudCoherence, "PointXYZRGBNormal");
}