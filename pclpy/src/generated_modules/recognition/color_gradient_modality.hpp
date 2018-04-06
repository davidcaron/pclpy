
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/color_gradient_modality.h>



template <typename PointInT>
void defineRecognitionColorGradientModality(py::module &m, std::string const & suffix) {
    using Class = ColorGradientModality<PointInT>;
    using PointCloudIn = Class::PointCloudIn;
    py::class_<Class, QuantizableModality, PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::FeatureSelectionMethod>(cls, "feature_selection_method")
        .value("MASK_BORDER_HIGH_GRADIENTS", Class::FeatureSelectionMethod::MASK_BORDER_HIGH_GRADIENTS)
        .value("MASK_BORDER_EQUALLY", Class::FeatureSelectionMethod::MASK_BORDER_EQUALLY)
        .value("DISTANCE_MAGNITUDE_SCORE", Class::FeatureSelectionMethod::DISTANCE_MAGNITUDE_SCORE)
        .export_values();
    cls.def(py::init<>());
    cls.def("set_gradient_magnitude_threshold", &Class::setGradientMagnitudeThreshold);
    cls.def("set_gradient_magnitude_threshold_for_feature_extraction", &Class::setGradientMagnitudeThresholdForFeatureExtraction);
    cls.def("set_feature_selection_method", &Class::setFeatureSelectionMethod);
    cls.def("set_spreading_size", &Class::setSpreadingSize);
    cls.def("set_variable_feature_nr", &Class::setVariableFeatureNr);
    cls.def("set_input_cloud", &Class::setInputCloud);
    cls.def("extract_features", &Class::extractFeatures);
    cls.def("extract_all_features", &Class::extractAllFeatures);
    cls.def("process_input_data", &Class::processInputData);
    cls.def("process_input_data_from_filtered", &Class::processInputDataFromFiltered);
        
}

void defineRecognitionColorGradientModalityClasses(py::module &sub_module) {
}