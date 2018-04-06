
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/quantized_map.h>



void defineRecognitionQuantizedMap(py::module &m) {
    using Class = QuantizedMap;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "QuantizedMap");
    cls.def(py::init<>());
    cls.def(py::init<size_t, size_t>(), "width"_a, "height"_a);
    cls.def_readonly("data_", &Class::data_);
    cls.def_readonly("width_", &Class::width_);
    cls.def_readonly("height_", &Class::height_);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("resize", &Class::resize);
    cls.def("spread_quantized_map", &Class::spreadQuantizedMap);
    cls.def("serialize", &Class::serialize);
    cls.def("deserialize", &Class::deserialize);
    cls.def("get_data", py::overload_cast<> (&Class::getData));
    cls.def("get_data", py::overload_cast<> (&Class::getData, py::const_));
}

void defineRecognitionQuantizedMapClasses(py::module &sub_module) {
    defineRecognitionQuantizedMap(sub_module);
}