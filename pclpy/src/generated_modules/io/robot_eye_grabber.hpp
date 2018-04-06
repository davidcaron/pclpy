
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/robot_eye_grabber.h>



void defineIoRobotEyeGrabber(py::module &m) {
    using Class = RobotEyeGrabber;
    py::class_<Class, Grabber, boost::shared_ptr<Class>> cls(m, "RobotEyeGrabber");
    cls.def(py::init<>());
    cls.def(py::init<boost::asio::ip::address, unsigned short>(), "ipAddress"_a, "port"_a=443);
    cls.def_property("sensor_address", &Class::getSensorAddress, &Class::setSensorAddress);
    cls.def_property("data_port", &Class::getDataPort, &Class::setDataPort);
    cls.def_property("signal_point_cloud_size", &Class::getSignalPointCloudSize, &Class::setSignalPointCloudSize);
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("is_running", &Class::isRunning);
}

void defineIoRobotEyeGrabberClasses(py::module &sub_module) {
    defineIoRobotEyeGrabber(sub_module);
}