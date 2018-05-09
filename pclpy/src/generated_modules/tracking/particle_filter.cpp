
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/tracking/particle_filter.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingParticleFilterTracker(py::module &m, std::string const & suffix) {
    using Class = pcl::tracking::ParticleFilterTracker<PointInT, StateT>;
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
    py::class_<Class, pcl::tracking::Tracker<PointInT, StateT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("toEigenMatrix", &Class::toEigenMatrix, "particle"_a);
    cls.def("normalizeParticleWeight", &Class::normalizeParticleWeight, "w"_a, "w_min"_a, "w_max"_a);
    cls.def("resetTracking", &Class::resetTracking);
    cls.def("setIterationNum", &Class::setIterationNum, "iteration_num"_a);
    cls.def("setParticleNum", &Class::setParticleNum, "particle_num"_a);
    cls.def("setReferenceCloud", &Class::setReferenceCloud, "ref"_a);
    cls.def("setCloudCoherence", &Class::setCloudCoherence, "coherence"_a);
    cls.def("setStepNoiseCovariance", &Class::setStepNoiseCovariance, "step_noise_covariance"_a);
    cls.def("setInitialNoiseCovariance", &Class::setInitialNoiseCovariance, "initial_noise_covariance"_a);
    cls.def("setInitialNoiseMean", &Class::setInitialNoiseMean, "initial_noise_mean"_a);
    cls.def("setResampleLikelihoodThr", &Class::setResampleLikelihoodThr, "resample_likelihood_thr"_a);
    cls.def("setOcclusionAngleThe", &Class::setOcclusionAngleThe, "occlusion_angle_thr"_a);
    cls.def("setMinIndices", &Class::setMinIndices, "min_indices"_a);
    cls.def("setTrans", &Class::setTrans, "trans"_a);
    cls.def("setAlpha", &Class::setAlpha, "alpha"_a);
    cls.def("setUseNormal", &Class::setUseNormal, "use_normal"_a);
    cls.def("setUseChangeDetector", &Class::setUseChangeDetector, "use_change_detector"_a);
    cls.def("setMotionRatio", &Class::setMotionRatio, "motion_ratio"_a);
    cls.def("setIntervalOfChangeDetection", &Class::setIntervalOfChangeDetection, "change_detector_interval"_a);
    cls.def("setMinPointsOfChangeDetection", &Class::setMinPointsOfChangeDetection, "change_detector_filter"_a);
    cls.def("setResolutionOfChangeDetection", &Class::setResolutionOfChangeDetection, "resolution"_a);
    cls.def("getIterationNum", &Class::getIterationNum);
    cls.def("getParticleNum", &Class::getParticleNum);
    cls.def("getReferenceCloud", &Class::getReferenceCloud);
    cls.def("getCloudCoherence", &Class::getCloudCoherence);
    cls.def("getTrans", &Class::getTrans);
    cls.def("getResult", &Class::getResult);
    cls.def("getParticles", &Class::getParticles);
    cls.def("getAlpha", &Class::getAlpha);
    cls.def("getUseNormal", &Class::getUseNormal);
    cls.def("getUseChangeDetector", &Class::getUseChangeDetector);
    cls.def("getMotionRatio", &Class::getMotionRatio);
    cls.def("getIntervalOfChangeDetection", &Class::getIntervalOfChangeDetection);
    cls.def("getResolutionOfChangeDetection", &Class::getResolutionOfChangeDetection);
    cls.def("getMinPointsOfChangeDetection", &Class::getMinPointsOfChangeDetection);
    cls.def("getFitRatio", &Class::getFitRatio);
        
}

void defineTrackingParticleFilterFunctions(py::module &m) {
}

void defineTrackingParticleFilterClasses(py::module &sub_module) {
    py::module sub_module_ParticleFilterTracker = sub_module.def_submodule("ParticleFilterTracker", "Submodule ParticleFilterTracker");
    defineTrackingParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::InterestPoint, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointNormal, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointWithRange, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointWithScale, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointWithViewpoint, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointXYZI, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointXYZINormal, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGB, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<pcl::PointXYZRGBNormal, pcl::tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}