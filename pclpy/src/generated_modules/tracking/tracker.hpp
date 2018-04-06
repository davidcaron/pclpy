
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/tracker.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingTracker(py::module &m, std::string const & suffix) {
    using Class = tracking::Tracker<PointInT, StateT>;
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
    py::class_<Class, PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("compute", &Class::compute);
        
}

void defineTrackingTrackerClasses(py::module &sub_module) {
    py::module sub_module_Tracker = sub_module.def_submodule("Tracker", "Submodule Tracker");
    defineTrackingTracker<InterestPoint, tracking::ParticleXYR>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingTracker<InterestPoint, tracking::ParticleXYRP>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingTracker<InterestPoint, tracking::ParticleXYRPY>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingTracker<InterestPoint, tracking::ParticleXYZR>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingTracker<InterestPoint, tracking::ParticleXYZRPY>(sub_module_Tracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointNormal, tracking::ParticleXYR>(sub_module_Tracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingTracker<PointNormal, tracking::ParticleXYRP>(sub_module_Tracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingTracker<PointNormal, tracking::ParticleXYRPY>(sub_module_Tracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingTracker<PointNormal, tracking::ParticleXYZR>(sub_module_Tracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingTracker<PointNormal, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointWithRange, tracking::ParticleXYR>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingTracker<PointWithRange, tracking::ParticleXYRP>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingTracker<PointWithRange, tracking::ParticleXYRPY>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingTracker<PointWithRange, tracking::ParticleXYZR>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingTracker<PointWithRange, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointWithScale, tracking::ParticleXYR>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingTracker<PointWithScale, tracking::ParticleXYRP>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingTracker<PointWithScale, tracking::ParticleXYRPY>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingTracker<PointWithScale, tracking::ParticleXYZR>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingTracker<PointWithScale, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointWithViewpoint, tracking::ParticleXYR>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingTracker<PointWithViewpoint, tracking::ParticleXYRP>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingTracker<PointWithViewpoint, tracking::ParticleXYRPY>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingTracker<PointWithViewpoint, tracking::ParticleXYZR>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingTracker<PointWithViewpoint, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointXYZ, tracking::ParticleXYR>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingTracker<PointXYZ, tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingTracker<PointXYZ, tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingTracker<PointXYZ, tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingTracker<PointXYZ, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointXYZI, tracking::ParticleXYR>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingTracker<PointXYZI, tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingTracker<PointXYZI, tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingTracker<PointXYZI, tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingTracker<PointXYZI, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointXYZINormal, tracking::ParticleXYR>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingTracker<PointXYZINormal, tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingTracker<PointXYZINormal, tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingTracker<PointXYZINormal, tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingTracker<PointXYZINormal, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointXYZRGB, tracking::ParticleXYR>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingTracker<PointXYZRGB, tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingTracker<PointXYZRGB, tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingTracker<PointXYZRGB, tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingTracker<PointXYZRGB, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointXYZRGBA, tracking::ParticleXYR>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingTracker<PointXYZRGBA, tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingTracker<PointXYZRGBA, tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingTracker<PointXYZRGBA, tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingTracker<PointXYZRGBA, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingTracker<PointXYZRGBNormal, tracking::ParticleXYR>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingTracker<PointXYZRGBNormal, tracking::ParticleXYRP>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingTracker<PointXYZRGBNormal, tracking::ParticleXYRPY>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingTracker<PointXYZRGBNormal, tracking::ParticleXYZR>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingTracker<PointXYZRGBNormal, tracking::ParticleXYZRPY>(sub_module_Tracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}