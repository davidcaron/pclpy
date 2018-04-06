
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/particle_filter_omp.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingParticleFilterOMPTracker(py::module &m, std::string const & suffix) {
    using Class = tracking::ParticleFilterOMPTracker<PointInT, StateT>;
    using BaseClass = Class::BaseClass;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    using PointCloudState = Class::PointCloudState;
    using PointCloudStatePtr = Class::PointCloudStatePtr;
    using PointCloudStateConstPtr = Class::PointCloudStateConstPtr;
    using Coherence = Class::Coherence;
    using CoherencePtr = Class::CoherencePtr;
    using CoherenceConstPtr = Class::CoherenceConstPtr;
    using CloudCoherence = Class::CloudCoherence;
    using CloudCoherencePtr = Class::CloudCoherencePtr;
    using CloudCoherenceConstPtr = Class::CloudCoherenceConstPtr;
    py::class_<Class, ParticleFilterTracker<PointInT,StateT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineTrackingParticleFilterOmpClasses(py::module &sub_module) {
    py::module sub_module_ParticleFilterOMPTracker = sub_module.def_submodule("ParticleFilterOMPTracker", "Submodule ParticleFilterOMPTracker");
    defineTrackingParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointNormal, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointNormal, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointNormal, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointNormal, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointNormal, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}