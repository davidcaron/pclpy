
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/sparse_quantized_multi_mod_template.h>



void defineRecognitionQuantizedMultiModFeature(py::module &m) {
    using Class = QuantizedMultiModFeature;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "QuantizedMultiModFeature");
    cls.def(py::init<>());
    cls.def_readonly("x", &Class::x);
    cls.def_readonly("y", &Class::y);
    cls.def_readonly("modality_index", &Class::modality_index);
    cls.def_readonly("quantized_value", &Class::quantized_value);
    cls.def("compare_for_equality", &Class::compareForEquality);
    cls.def("serialize", &Class::serialize);
    cls.def("deserialize", &Class::deserialize);
}

void defineRecognitionSparseQuantizedMultiModTemplate(py::module &m) {
    using Class = SparseQuantizedMultiModTemplate;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "SparseQuantizedMultiModTemplate");
    cls.def(py::init<>());
    cls.def_readonly("features", &Class::features);
    cls.def_readonly("region", &Class::region);
    cls.def("serialize", &Class::serialize);
    cls.def("deserialize", &Class::deserialize);
}

void defineRecognitionSparseQuantizedMultiModTemplateClasses(py::module &sub_module) {
    defineRecognitionQuantizedMultiModFeature(sub_module);
    defineRecognitionSparseQuantizedMultiModTemplate(sub_module);
}