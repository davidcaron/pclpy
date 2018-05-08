
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

#include <pcl/visualization/keyboard_event.h>

using namespace pcl::visualization;


void defineVisualizationKeyboardEvent(py::module &m) {
    using Class = pcl::visualization::KeyboardEvent;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "KeyboardEvent");
    cls.def(py::init<bool, std::string, unsigned char, bool, bool, bool>(), "action"_a, "key_sym"_a, "key"_a, "alt"_a, "ctrl"_a, "shift"_a);
    cls.def_readonly_static("Alt", &Class::Alt);
    cls.def_readonly_static("Ctrl", &Class::Ctrl);
    cls.def_readonly_static("Shift", &Class::Shift);
    cls.def("isAltPressed", &Class::isAltPressed);
    cls.def("isCtrlPressed", &Class::isCtrlPressed);
    cls.def("isShiftPressed", &Class::isShiftPressed);
    cls.def("keyDown", &Class::keyDown);
    cls.def("keyUp", &Class::keyUp);
    cls.def("getKeyCode", &Class::getKeyCode);
    cls.def("getKeySym", &Class::getKeySym);
}

void defineVisualizationKeyboardEventFunctions(py::module &m) {
}

void defineVisualizationKeyboardEventClasses(py::module &sub_module) {
    defineVisualizationKeyboardEvent(sub_module);
}