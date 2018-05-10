
#include <pcl/recognition/sparse_quantized_multi_mod_template.h>



void defineRecognitionQuantizedMultiModFeature(py::module &m) {
    using Class = pcl::QuantizedMultiModFeature;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "QuantizedMultiModFeature");
    cls.def(py::init<>());
    cls.def_readwrite("x", &Class::x);
    cls.def_readwrite("y", &Class::y);
    cls.def_readwrite("modality_index", &Class::modality_index);
    cls.def_readwrite("quantized_value", &Class::quantized_value);
    cls.def("compareForEquality", &Class::compareForEquality, "base"_a);
    cls.def("serialize", &Class::serialize, "stream"_a);
    cls.def("deserialize", &Class::deserialize, "stream"_a);
}

void defineRecognitionSparseQuantizedMultiModTemplate(py::module &m) {
    using Class = pcl::SparseQuantizedMultiModTemplate;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "SparseQuantizedMultiModTemplate");
    cls.def(py::init<>());
    cls.def_readwrite("features", &Class::features);
    cls.def_readwrite("region", &Class::region);
    cls.def("serialize", &Class::serialize, "stream"_a);
    cls.def("deserialize", &Class::deserialize, "stream"_a);
}

void defineRecognitionSparseQuantizedMultiModTemplateFunctions(py::module &m) {
}

void defineRecognitionSparseQuantizedMultiModTemplateClasses(py::module &sub_module) {
    defineRecognitionQuantizedMultiModFeature(sub_module);
    defineRecognitionSparseQuantizedMultiModTemplate(sub_module);
}