
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