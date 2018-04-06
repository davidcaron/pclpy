
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/hdl_grabber.h>



void defineIoHDLGrabber(py::module &m) {
    using Class = HDLGrabber;
    py::class_<Class, Grabber, boost::shared_ptr<Class>> cls(m, "HDLGrabber");
    cls.def(py::init<std::string, std::string>(), "correctionsFile"_a="", "pcapFile"_a="");
    cls.def(py::init<boost::asio::ip::address, unsigned short, std::string>(), "ipAddress"_a, "port"_a, "correctionsFile"_a="");
    cls.def("set_laser_color_rgb", &Class::setLaserColorRGB);
    cls.def_property("minimum_distance_threshold", &Class::getMinimumDistanceThreshold, &Class::setMinimumDistanceThreshold);
    cls.def_property("maximum_distance_threshold", &Class::getMaximumDistanceThreshold, &Class::setMaximumDistanceThreshold);
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("is_running", &Class::isRunning);
    cls.def("filter_packets", &Class::filterPackets);
}

void defineIoHdlGrabberClasses(py::module &sub_module) {
    defineIoHDLGrabber(sub_module);
}