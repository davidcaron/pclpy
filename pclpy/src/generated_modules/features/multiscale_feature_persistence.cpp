
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/multiscale_feature_persistence.h>



template <typename PointSource, typename PointFeature>
void defineFeaturesMultiscaleFeaturePersistence(py::module &m, std::string const & suffix) {
    using Class = pcl::MultiscaleFeaturePersistence<PointSource, PointFeature>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using FeatureCloud = Class::FeatureCloud;
    using FeatureCloudPtr = Class::FeatureCloudPtr;
    using FeatureEstimatorPtr = Class::FeatureEstimatorPtr;
    using FeatureRepresentationConstPtr = Class::FeatureRepresentationConstPtr;
    py::class_<Class, pcl::PCLBase<PointSource>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computeFeaturesAtAllScales", &Class::computeFeaturesAtAllScales);
    cls.def("determinePersistentFeatures", &Class::determinePersistentFeatures, "output_features"_a, "output_indices"_a);
    cls.def("setScalesVector", &Class::setScalesVector, "scale_values"_a);
    cls.def("setFeatureEstimator", &Class::setFeatureEstimator, "feature_estimator"_a);
    cls.def("setPointRepresentation", &Class::setPointRepresentation, "feature_representation"_a);
    cls.def("setAlpha", &Class::setAlpha, "alpha"_a);
    cls.def("setDistanceMetric", &Class::setDistanceMetric, "distance_metric"_a);
    cls.def("getScalesVector", &Class::getScalesVector);
    cls.def("getFeatureEstimator", &Class::getFeatureEstimator);
    cls.def("getPointRepresentation", &Class::getPointRepresentation);
    cls.def("getAlpha", &Class::getAlpha);
    cls.def("getDistanceMetric", &Class::getDistanceMetric);
        
}

void defineFeaturesMultiscaleFeaturePersistenceFunctions(py::module &m) {
}

void defineFeaturesMultiscaleFeaturePersistenceClasses(py::module &sub_module) {
    py::module sub_module_MultiscaleFeaturePersistence = sub_module.def_submodule("MultiscaleFeaturePersistence", "Submodule MultiscaleFeaturePersistence");
    defineFeaturesMultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33>(sub_module_MultiscaleFeaturePersistence, "PointXYZ_FPFHSignature33");
    defineFeaturesMultiscaleFeaturePersistence<pcl::PointXYZRGBA, pcl::FPFHSignature33>(sub_module_MultiscaleFeaturePersistence, "PointXYZRGBA_FPFHSignature33");
}