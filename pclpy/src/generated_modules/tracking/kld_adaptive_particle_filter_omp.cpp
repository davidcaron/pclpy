
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingKLDAdaptiveParticleFilterOMPTracker(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointInT, StateT>;
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
    py::class_<Class, pcl::tracking::KLDAdaptiveParticleFilterTracker<PointInT, StateT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineTrackingKldAdaptiveParticleFilterOmpFunctions(py::module &m) {
}

void defineTrackingKldAdaptiveParticleFilterOmpClasses(py::module &sub_module) {
    py::module sub_module_KLDAdaptiveParticleFilterOMPTracker = sub_module.def_submodule("KLDAdaptiveParticleFilterOMPTracker", "Submodule KLDAdaptiveParticleFilterOMPTracker");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRP>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZR>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingKLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZRPY>(sub_module_KLDAdaptiveParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}