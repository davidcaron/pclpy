
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

#include <pcl/recognition/dense_quantized_multi_mod_template.h>



void defineRecognitionDenseQuantizedMultiModTemplate(py::module &m) {
    using Class = pcl::DenseQuantizedMultiModTemplate;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DenseQuantizedMultiModTemplate");
    cls.def_readwrite("modalities", &Class::modalities);
    cls.def_readwrite("response_factor", &Class::response_factor);
    cls.def_readwrite("region", &Class::region);
    cls.def("serialize", &Class::serialize, "stream"_a);
    cls.def("deserialize", &Class::deserialize, "stream"_a);
}

void defineRecognitionDenseQuantizedSingleModTemplate(py::module &m) {
    using Class = pcl::DenseQuantizedSingleModTemplate;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DenseQuantizedSingleModTemplate");
    cls.def_readwrite("features", &Class::features);
    cls.def("serialize", &Class::serialize, "stream"_a);
    cls.def("deserialize", &Class::deserialize, "stream"_a);
}

void defineRecognitionDenseQuantizedMultiModTemplateFunctions(py::module &m) {
}

void defineRecognitionDenseQuantizedMultiModTemplateClasses(py::module &sub_module) {
    defineRecognitionDenseQuantizedMultiModTemplate(sub_module);
    defineRecognitionDenseQuantizedSingleModTemplate(sub_module);
}