
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/time.h>



void defineCommonEventFrequency(py::module &m) {
    using Class = EventFrequency;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "EventFrequency");
    cls.def(py::init<size_t>(), "window_size"_a=30);
    cls.def("event", &Class::event);
    cls.def("reset", &Class::reset);
}

void defineCommonStopWatch(py::module &m) {
    using Class = StopWatch;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "StopWatch");
    cls.def(py::init<>());
    cls.def("reset", &Class::reset);
}

void defineCommonScopeTime(py::module &m) {
    using Class = ScopeTime;
    py::class_<Class, StopWatch, boost::shared_ptr<Class>> cls(m, "ScopeTime");
    cls.def(py::init<char*>(), "title"_a);
    cls.def(py::init<>());
}

void defineCommonTimeClasses(py::module &sub_module) {
    defineCommonEventFrequency(sub_module);
    defineCommonStopWatch(sub_module);
    defineCommonScopeTime(sub_module);
}