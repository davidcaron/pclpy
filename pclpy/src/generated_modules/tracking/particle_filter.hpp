
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/tracking/particle_filter.h>

using namespace pcl::tracking;


template <typename PointInT, typename StateT>
void defineTrackingParticleFilterTracker(py::module &m, std::string const & suffix) {
    using Class = tracking::ParticleFilterTracker<PointInT, StateT>;
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
    py::class_<Class, Tracker<PointInT,StateT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("iteration_num", &Class::getIterationNum, &Class::setIterationNum);
    cls.def_property("particle_num", &Class::getParticleNum, &Class::setParticleNum);
    cls.def_property("reference_cloud", &Class::getReferenceCloud, &Class::setReferenceCloud);
    cls.def_property("cloud_coherence", &Class::getCloudCoherence, &Class::setCloudCoherence);
    cls.def("set_step_noise_covariance", &Class::setStepNoiseCovariance);
    cls.def("set_initial_noise_covariance", &Class::setInitialNoiseCovariance);
    cls.def("set_initial_noise_mean", &Class::setInitialNoiseMean);
    cls.def("set_resample_likelihood_thr", &Class::setResampleLikelihoodThr);
    cls.def("set_occlusion_angle_the", &Class::setOcclusionAngleThe);
    cls.def("set_min_indices", &Class::setMinIndices);
    cls.def_property("trans", &Class::getTrans, &Class::setTrans);
    cls.def_property("alpha", &Class::getAlpha, &Class::setAlpha);
    cls.def_property("use_normal", &Class::getUseNormal, &Class::setUseNormal);
    cls.def_property("use_change_detector", &Class::getUseChangeDetector, &Class::setUseChangeDetector);
    cls.def_property("motion_ratio", &Class::getMotionRatio, &Class::setMotionRatio);
    cls.def_property("interval_of_change_detection", &Class::getIntervalOfChangeDetection, &Class::setIntervalOfChangeDetection);
    cls.def_property("min_points_of_change_detection", &Class::getMinPointsOfChangeDetection, &Class::setMinPointsOfChangeDetection);
    cls.def_property("resolution_of_change_detection", &Class::getResolutionOfChangeDetection, &Class::setResolutionOfChangeDetection);
    cls.def("to_eigen_matrix", &Class::toEigenMatrix);
    cls.def("normalize_particle_weight", &Class::normalizeParticleWeight);
    cls.def("reset_tracking", &Class::resetTracking);
        
}

void defineTrackingParticleFilterClasses(py::module &sub_module) {
    py::module sub_module_ParticleFilterTracker = sub_module.def_submodule("ParticleFilterTracker", "Submodule ParticleFilterTracker");
    defineTrackingParticleFilterTracker<InterestPoint, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<InterestPoint, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<InterestPoint, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<InterestPoint, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<InterestPoint, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "InterestPoint_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointNormal, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointNormal, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointNormal, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointNormal, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointNormal, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointNormal_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointWithRange, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointWithRange, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointWithRange, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointWithRange, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointWithRange, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointWithRange_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointWithScale, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointWithScale, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointWithScale, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointWithScale, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointWithScale, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointWithScale_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointWithViewpoint, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointWithViewpoint_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointXYZ, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointXYZ, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointXYZ, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointXYZ, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointXYZ, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZ_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointXYZI, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointXYZI, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointXYZI, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointXYZI, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointXYZI, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZI_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointXYZINormal, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointXYZINormal, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointXYZINormal, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointXYZINormal, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointXYZINormal, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZINormal_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointXYZRGB, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointXYZRGB, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointXYZRGB, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointXYZRGB, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointXYZRGB, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZRGB_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointXYZRGBA, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZRGBA_tracking::ParticleXYZRPY");
    defineTrackingParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYR>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYR");
    defineTrackingParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYRP>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYRP");
    defineTrackingParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYRPY>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYRPY");
    defineTrackingParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYZR>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYZR");
    defineTrackingParticleFilterTracker<PointXYZRGBNormal, tracking::ParticleXYZRPY>(sub_module_ParticleFilterTracker, "PointXYZRGBNormal_tracking::ParticleXYZRPY");
}