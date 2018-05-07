
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/recognition/point_types.h>



void defineRecognitionGradientXY(py::module &m) {
    using Class = pcl::GradientXY;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "GradientXY");
    cls.def_readonly("data", &Class::data);
    // Operators not implemented (operator<);
}

void defineRecognitionPointTypesFunctions(py::module &m) {
}

void defineRecognitionPointTypesClasses(py::module &sub_module) {
    defineRecognitionGradientXY(sub_module);
    defineRecognitionPointTypesFunctions(sub_module);
}