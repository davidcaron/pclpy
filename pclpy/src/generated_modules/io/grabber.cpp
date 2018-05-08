
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

#include <pcl/io/grabber.h>



void defineIoGrabber(py::module &m) {
    using Class = pcl::Grabber;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Grabber");
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("isRunning", &Class::isRunning);
    cls.def("getName", &Class::getName);
    cls.def("getFramesPerSecond", &Class::getFramesPerSecond);
}

void defineIoGrabberFunctions(py::module &m) {
}

void defineIoGrabberClasses(py::module &sub_module) {
    defineIoGrabber(sub_module);
}