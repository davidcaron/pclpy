
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/tracking/particle_filter_omp.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingParticleFilterOMPTracker(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::ParticleFilterOMPTracker<PointInT, StateT>;
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
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
        
}

void defineTrackingParticleFilterOmpFunctions(py::module &m) {
}

void defineTrackingParticleFilterOmpClasses(py::module &sub_module) {
    py::module sub_module_ParticleFilterOMPTracker = sub_module.def_submodule("ParticleFilterOMPTracker", "Submodule ParticleFilterOMPTracker");
    defineTrackingParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointNormal, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterOMPTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterOMPTracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}