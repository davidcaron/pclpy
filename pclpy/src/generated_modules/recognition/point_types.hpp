
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/point_types.h>



void defineRecognitionGradientXY(py::module &m) {
    using Class = GradientXY;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "GradientXY");
    cls.def_readonly("data", &Class::data);
    // Operators not implemented (operator<);
}

void defineRecognitionPointTypesClasses(py::module &sub_module) {
    defineRecognitionGradientXY(sub_module);
}