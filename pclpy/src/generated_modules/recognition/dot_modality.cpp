
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/recognition/dot_modality.h>



void defineRecognitionDOTModality(py::module &m) {
    using Class = pcl::DOTModality;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DOTModality");
    cls.def("computeInvariantQuantizedMap", &Class::computeInvariantQuantizedMap, "mask"_a, "region"_a);
    cls.def("getDominantQuantizedMap", &Class::getDominantQuantizedMap);
}

void defineRecognitionDotModalityFunctions(py::module &m) {
}

void defineRecognitionDotModalityClasses(py::module &sub_module) {
    defineRecognitionDOTModality(sub_module);
}