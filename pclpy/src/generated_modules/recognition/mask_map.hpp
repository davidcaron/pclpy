
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/mask_map.h>



void defineRecognitionMaskMap(py::module &m) {
    using Class = MaskMap;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "MaskMap");
    cls.def(py::init<>());
    cls.def(py::init<size_t, size_t>(), "width"_a, "height"_a);
    cls.def("set_", &Class::set);
    // Operators not implemented (operator());
    // Operators not implemented (operator());
    cls.def("resize", &Class::resize);
    cls.def("unset", &Class::unset);
    cls.def("is_set", &Class::isSet);
    cls.def("reset", &Class::reset);
    cls.def("erode", &Class::erode);
    cls.def("get_data", py::overload_cast<> (&Class::getData));
    cls.def("get_data", py::overload_cast<> (&Class::getData, py::const_));
}

void defineRecognitionMaskMapClasses(py::module &sub_module) {
    defineRecognitionMaskMap(sub_module);
}