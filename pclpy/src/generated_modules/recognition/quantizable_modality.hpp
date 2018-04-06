
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/quantizable_modality.h>



void defineRecognitionQuantizableModality(py::module &m) {
    using Class = QuantizableModality;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "QuantizableModality");
    cls.def("extract_features", &Class::extractFeatures);
    cls.def("extract_all_features", &Class::extractAllFeatures);
}

void defineRecognitionQuantizableModalityClasses(py::module &sub_module) {
    defineRecognitionQuantizableModality(sub_module);
}