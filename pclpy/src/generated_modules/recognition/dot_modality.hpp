
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/dot_modality.h>



void defineRecognitionDOTModality(py::module &m) {
    using Class = DOTModality;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DOTModality");
    cls.def("compute_invariant_quantized_map", &Class::computeInvariantQuantizedMap);
}

void defineRecognitionDotModalityClasses(py::module &sub_module) {
    defineRecognitionDOTModality(sub_module);
}