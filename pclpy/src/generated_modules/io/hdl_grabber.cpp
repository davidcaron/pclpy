
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/io/hdl_grabber.h>



void defineIoHDLGrabber(py::module &m) {
    using Class = pcl::HDLGrabber;
    py::class_<Class, pcl::Grabber, boost::shared_ptr<Class>> cls(m, "HDLGrabber");
    cls.def(py::init<std::string, std::string>(), "correctionsFile"_a="", "pcapFile"_a="");
    cls.def(py::init<boost::asio::ip::address, unsigned short, std::string>(), "ipAddress"_a, "port"_a, "correctionsFile"_a="");
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("isRunning", &Class::isRunning);
    cls.def("filterPackets", &Class::filterPackets, "ipAddress"_a, "port"_a=443);
    cls.def("setLaserColorRGB", &Class::setLaserColorRGB, "color"_a, "laserNumber"_a);
    cls.def("setMinimumDistanceThreshold", &Class::setMinimumDistanceThreshold, "minThreshold"_a);
    cls.def("setMaximumDistanceThreshold", &Class::setMaximumDistanceThreshold, "maxThreshold"_a);
    cls.def("getName", &Class::getName);
    cls.def("getFramesPerSecond", &Class::getFramesPerSecond);
    cls.def("getMinimumDistanceThreshold", &Class::getMinimumDistanceThreshold);
    cls.def("getMaximumDistanceThreshold", &Class::getMaximumDistanceThreshold);
}

void defineIoHdlGrabberFunctions(py::module &m) {
}

void defineIoHdlGrabberClasses(py::module &sub_module) {
    defineIoHDLGrabber(sub_module);
}