
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

#include <pcl/common/time.h>



void defineCommonEventFrequency(py::module &m) {
    using Class = pcl::EventFrequency;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "EventFrequency");
    cls.def(py::init<size_t>(), "window_size"_a=30);
    cls.def("event", &Class::event);
    cls.def("reset", &Class::reset);
    cls.def("getFrequency", &Class::getFrequency);
}

void defineCommonStopWatch(py::module &m) {
    using Class = pcl::StopWatch;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "StopWatch");
    cls.def(py::init<>());
    cls.def("reset", &Class::reset);
    cls.def("getTime", &Class::getTime);
    cls.def("getTimeSeconds", &Class::getTimeSeconds);
}

void defineCommonScopeTime(py::module &m) {
    using Class = pcl::ScopeTime;
    py::class_<Class, pcl::StopWatch, boost::shared_ptr<Class>> cls(m, "ScopeTime");
    cls.def(py::init<char*>(), "title"_a);
    cls.def(py::init<>());
}

void defineCommonTimeFunctions1(py::module &m) {
    m.def("getTime", py::overload_cast<> (&pcl::getTime));
}

void defineCommonTimeFunctions(py::module &m) {
    defineCommonTimeFunctions1(m);
}

void defineCommonTimeClasses(py::module &sub_module) {
    defineCommonEventFrequency(sub_module);
    defineCommonStopWatch(sub_module);
    defineCommonScopeTime(sub_module);
    defineCommonTimeFunctions(sub_module);
}