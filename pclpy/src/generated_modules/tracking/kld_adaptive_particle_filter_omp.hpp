
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingKLDAdaptiveParticleFilterOMPTracker(py::module &m, std::string const & suffix) {
    using Class = tracking::KLDAdaptiveParticleFilterOMPTracker<PointInT, StateT>;
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
    py::class_<Class, KLDAdaptiveParticleFilterTracker<PointInT,StateT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
        
}

void defineTrackingKldAdaptiveParticleFilterOmpClasses(py::module &sub_module) {
    py::module sub_module_KLDAdaptiveParticleFilterOMPTracker = sub_module.def_submodule("KLDAdaptiveParticleFilterOMPTracker", "Submodule KLDAdaptiveParticleFilterOMPTracker");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<InterestPoint, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointNormal, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointNormal, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointNormal, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointNormal, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointNormal, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithRange, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithScale, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointWithViewpoint, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZ, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZI, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZINormal, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGB, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBA, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<PointXYZRGBNormal, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}