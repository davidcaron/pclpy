
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/dense_quantized_multi_mod_template.h>



void defineRecognitionDenseQuantizedMultiModTemplate(py::module &m) {
    using Class = DenseQuantizedMultiModTemplate;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DenseQuantizedMultiModTemplate");
    cls.def_readonly("modalities", &Class::modalities);
    cls.def_readonly("response_factor", &Class::response_factor);
    cls.def_readonly("region", &Class::region);
    cls.def("serialize", &Class::serialize);
    cls.def("deserialize", &Class::deserialize);
}

void defineRecognitionDenseQuantizedSingleModTemplate(py::module &m) {
    using Class = DenseQuantizedSingleModTemplate;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DenseQuantizedSingleModTemplate");
    cls.def_readonly("features", &Class::features);
    cls.def("serialize", &Class::serialize);
    cls.def("deserialize", &Class::deserialize);
}

void defineRecognitionDenseQuantizedMultiModTemplateClasses(py::module &sub_module) {
    defineRecognitionDenseQuantizedMultiModTemplate(sub_module);
    defineRecognitionDenseQuantizedSingleModTemplate(sub_module);
}