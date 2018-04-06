
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/multiscale_feature_persistence.h>



template <typename PointSource, typename PointFeature>
void defineFeaturesMultiscaleFeaturePersistence(py::module &m, std::string const & suffix) {
    using Class = MultiscaleFeaturePersistence<PointSource, PointFeature>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using FeatureCloud = Class::FeatureCloud;
    using FeatureCloudPtr = Class::FeatureCloudPtr;
    using FeatureEstimatorPtr = Class::FeatureEstimatorPtr;
    using FeatureRepresentationConstPtr = Class::FeatureRepresentationConstPtr;
    py::class_<Class, PCLBase<PointSource>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("scales_vector", &Class::getScalesVector, &Class::setScalesVector);
    cls.def_property("feature_estimator", &Class::getFeatureEstimator, &Class::setFeatureEstimator);
    cls.def_property("point_representation", &Class::getPointRepresentation, &Class::setPointRepresentation);
    cls.def_property("alpha", &Class::getAlpha, &Class::setAlpha);
    cls.def_property("distance_metric", &Class::getDistanceMetric, &Class::setDistanceMetric);
    cls.def("compute_features_at_all_scales", &Class::computeFeaturesAtAllScales);
    cls.def("determine_persistent_features", &Class::determinePersistentFeatures);
        
}

void defineFeaturesMultiscaleFeaturePersistenceClasses(py::module &sub_module) {
    py::module sub_module_MultiscaleFeaturePersistence = sub_module.def_submodule("MultiscaleFeaturePersistence", "Submodule MultiscaleFeaturePersistence");
    defineFeaturesMultiscaleFeaturePersistence<PointXYZ, FPFHSignature33>(sub_module_MultiscaleFeaturePersistence, "PointXYZ_FPFHSignature33");
    defineFeaturesMultiscaleFeaturePersistence<PointXYZRGBA, FPFHSignature33>(sub_module_MultiscaleFeaturePersistence, "PointXYZRGBA_FPFHSignature33");
}