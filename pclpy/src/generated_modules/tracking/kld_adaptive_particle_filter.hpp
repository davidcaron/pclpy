
#include <pcl/tracking/kld_adaptive_particle_filter.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingKLDAdaptiveParticleFilterTracker(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::KLDAdaptiveParticleFilterTracker<PointInT, StateT>;
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
    py::class_<Class, pcl::tracking::ParticleFilterTracker<PointInT, StateT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setBinSize", &Class::setBinSize, "bin_size"_a);
    cls.def("setMaximumParticleNum", &Class::setMaximumParticleNum, "nr"_a);
    cls.def("setEpsilon", &Class::setEpsilon, "eps"_a);
    cls.def("setDelta", &Class::setDelta, "delta"_a);
    cls.def("getBinSize", &Class::getBinSize);
    cls.def("getMaximumParticleNum", &Class::getMaximumParticleNum);
    cls.def("getEpsilon", &Class::getEpsilon);
    cls.def("getDelta", &Class::getDelta);
        
}

void defineTrackingKldAdaptiveParticleFilterFunctions(py::module &m) {
}

void defineTrackingKldAdaptiveParticleFilterClasses(py::module &sub_module) {
    py::module sub_module_KLDAdaptiveParticleFilterTracker = sub_module.def_submodule("KLDAdaptiveParticleFilterTracker", "Submodule KLDAdaptiveParticleFilterTracker");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}