
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/kld_adaptive_particle_filter.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingKLDAdaptiveParticleFilterTracker(py::module &m, std::string const & suffix) {
    using Class = tracking::KLDAdaptiveParticleFilterTracker<PointInT, StateT>;
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
    cls.def_property("bin_size", &Class::getBinSize, &Class::setBinSize);
    cls.def_property("maximum_particle_num", &Class::getMaximumParticleNum, &Class::setMaximumParticleNum);
    cls.def_property("epsilon", &Class::getEpsilon, &Class::setEpsilon);
    cls.def_property("delta", &Class::getDelta, &Class::setDelta);
        
}

void defineTrackingKldAdaptiveParticleFilterClasses(py::module &sub_module) {
    py::module sub_module_KLDAdaptiveParticleFilterTracker = sub_module.def_submodule("KLDAdaptiveParticleFilterTracker", "Submodule KLDAdaptiveParticleFilterTracker");
    defineTrackingKLDAdaptiveParticleFilterTracker<InterestPoint, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<InterestPoint, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<InterestPoint, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<InterestPoint, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<InterestPoint, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointNormal, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointNormal, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointNormal, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointNormal, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointNormal, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithRange, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithRange, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithRange, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithRange, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithRange, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithScale, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithScale, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithScale, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithScale, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithScale, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZ, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZ, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZ, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZ, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZ, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZI, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZI, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZI, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZI, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZI, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZINormal, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZINormal, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZINormal, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZINormal, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZINormal, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGB, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGB, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGB, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGB, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGB, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}