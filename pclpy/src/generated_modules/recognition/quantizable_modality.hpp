
#include <pcl/recognition/quantizable_modality.h>



void defineRecognitionQuantizableModality(py::module &m) {
    using Class = pcl::QuantizableModality;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "QuantizableModality");
    cls.def("extractFeatures", &Class::extractFeatures, "mask"_a, "nr_features"_a, "modality_index"_a, "features"_a);
    cls.def("extractAllFeatures", &Class::extractAllFeatures, "mask"_a, "nr_features"_a, "modality_index"_a, "features"_a);
    cls.def("getQuantizedMap", &Class::getQuantizedMap);
    cls.def("getSpreadedQuantizedMap", &Class::getSpreadedQuantizedMap);
}

void defineRecognitionQuantizableModalityFunctions(py::module &m) {
}

void defineRecognitionQuantizableModalityClasses(py::module &sub_module) {
    defineRecognitionQuantizableModality(sub_module);
}