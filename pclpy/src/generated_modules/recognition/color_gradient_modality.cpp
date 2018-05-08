
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/recognition/color_gradient_modality.h>



template <typename PointInT>
void defineRecognitionColorGradientModality(py::module &m, std::string const & suffix) {
    using Class = pcl::ColorGradientModality<PointInT>;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, pcl::QuantizableModality, pcl::PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::FeatureSelectionMethod>(cls, "FeatureSelectionMethod")
        .value("MASK_BORDER_HIGH_GRADIENTS", Class::FeatureSelectionMethod::MASK_BORDER_HIGH_GRADIENTS)
        .value("MASK_BORDER_EQUALLY", Class::FeatureSelectionMethod::MASK_BORDER_EQUALLY)
        .value("DISTANCE_MAGNITUDE_SCORE", Class::FeatureSelectionMethod::DISTANCE_MAGNITUDE_SCORE)
        .export_values();
    cls.def(py::init<>());
    cls.def("extractFeatures", &Class::extractFeatures, "mask"_a, "nr_features"_a, "modalityIndex"_a, "features"_a);
    cls.def("extractAllFeatures", &Class::extractAllFeatures, "mask"_a, "nr_features"_a, "modalityIndex"_a, "features"_a);
    cls.def("processInputData", &Class::processInputData);
    cls.def("processInputDataFromFiltered", &Class::processInputDataFromFiltered);
    cls.def("setGradientMagnitudeThreshold", &Class::setGradientMagnitudeThreshold, "threshold"_a);
    cls.def("setGradientMagnitudeThresholdForFeatureExtraction", &Class::setGradientMagnitudeThresholdForFeatureExtraction, "threshold"_a);
    cls.def("setFeatureSelectionMethod", &Class::setFeatureSelectionMethod, "method"_a);
    cls.def("setSpreadingSize", &Class::setSpreadingSize, "spreading_size"_a);
    cls.def("setVariableFeatureNr", &Class::setVariableFeatureNr, "enabled"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getQuantizedMap", &Class::getQuantizedMap);
    cls.def("getSpreadedQuantizedMap", &Class::getSpreadedQuantizedMap);
    cls.def("getMaxColorGradients", &Class::getMaxColorGradients);
        
}

void defineRecognitionColorGradientModalityFunctions(py::module &m) {
}

void defineRecognitionColorGradientModalityClasses(py::module &sub_module) {
}