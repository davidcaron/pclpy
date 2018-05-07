
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/tracking/tracker.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingTracker(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::Tracker<PointInT, StateT>;
    using BaseClass = Class::BaseClass;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using SearchPtr = Class::SearchPtr;
    using SearchConstPtr = Class::SearchConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudState = Class::PointCloudState;
    using PointCloudStatePtr = Class::PointCloudStatePtr;
    using PointCloudStateConstPtr = Class::PointCloudStateConstPtr;
    py::class_<Class, pcl::PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("compute", &Class::compute);
        
}

void defineTrackingTrackerFunctions(py::module &m) {
}

void defineTrackingTrackerClasses(py::module &sub_module) {
    py::module sub_module_Tracker = sub_module.def_submodule("Tracker", "Submodule Tracker");
    defineTrackingTracker<pcl::InterestPoint, pcl::tracking::ParticleXYR>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointNormal, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointNormal, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointNormal, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointNormal, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointNormal, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointWithRange, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointWithScale, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointXYZ, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointXYZI, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYR>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}