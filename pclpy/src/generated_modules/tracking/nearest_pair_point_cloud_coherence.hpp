
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

using namespace pcl::tracking;


template <typename PointInT>
void defineTrackingNearestPairPointCloudCoherence(py::module &m, std::string const & suffix) {
    using Class = tracking::NearestPairPointCloudCoherence<PointInT>;
    using PointCoherencePtr = Class::PointCoherencePtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using BaseClass = Class::BaseClass;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using SearchPtr = Class::SearchPtr;
    using SearchConstPtr = Class::SearchConstPtr;
    py::class_<Class, PointCloudCoherence<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def("set_target_cloud", &Class::setTargetCloud);
    cls.def("set_maximum_distance", &Class::setMaximumDistance);
        
}

void defineTrackingNearestPairPointCloudCoherenceClasses(py::module &sub_module) {
    py::module sub_module_NearestPairPointCloudCoherence = sub_module.def_submodule("NearestPairPointCloudCoherence", "Submodule NearestPairPointCloudCoherence");
    defineTrackingNearestPairPointCloudCoherence<InterestPoint>(sub_module_NearestPairPointCloudCoherence, "InterestPoint");
    defineTrackingNearestPairPointCloudCoherence<PointDEM>(sub_module_NearestPairPointCloudCoherence, "PointDEM");
    defineTrackingNearestPairPointCloudCoherence<PointNormal>(sub_module_NearestPairPointCloudCoherence, "PointNormal");
    defineTrackingNearestPairPointCloudCoherence<PointSurfel>(sub_module_NearestPairPointCloudCoherence, "PointSurfel");
    defineTrackingNearestPairPointCloudCoherence<PointWithRange>(sub_module_NearestPairPointCloudCoherence, "PointWithRange");
    defineTrackingNearestPairPointCloudCoherence<PointWithScale>(sub_module_NearestPairPointCloudCoherence, "PointWithScale");
    defineTrackingNearestPairPointCloudCoherence<PointWithViewpoint>(sub_module_NearestPairPointCloudCoherence, "PointWithViewpoint");
    defineTrackingNearestPairPointCloudCoherence<PointXYZ>(sub_module_NearestPairPointCloudCoherence, "PointXYZ");
    defineTrackingNearestPairPointCloudCoherence<PointXYZHSV>(sub_module_NearestPairPointCloudCoherence, "PointXYZHSV");
    defineTrackingNearestPairPointCloudCoherence<PointXYZI>(sub_module_NearestPairPointCloudCoherence, "PointXYZI");
    defineTrackingNearestPairPointCloudCoherence<PointXYZINormal>(sub_module_NearestPairPointCloudCoherence, "PointXYZINormal");
    defineTrackingNearestPairPointCloudCoherence<PointXYZL>(sub_module_NearestPairPointCloudCoherence, "PointXYZL");
    defineTrackingNearestPairPointCloudCoherence<PointXYZLNormal>(sub_module_NearestPairPointCloudCoherence, "PointXYZLNormal");
    defineTrackingNearestPairPointCloudCoherence<PointXYZRGB>(sub_module_NearestPairPointCloudCoherence, "PointXYZRGB");
    defineTrackingNearestPairPointCloudCoherence<PointXYZRGBA>(sub_module_NearestPairPointCloudCoherence, "PointXYZRGBA");
    defineTrackingNearestPairPointCloudCoherence<PointXYZRGBL>(sub_module_NearestPairPointCloudCoherence, "PointXYZRGBL");
    defineTrackingNearestPairPointCloudCoherence<PointXYZRGBNormal>(sub_module_NearestPairPointCloudCoherence, "PointXYZRGBNormal");
}