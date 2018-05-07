
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/io/robot_eye_grabber.h>



void defineIoRobotEyeGrabber(py::module &m) {
    using Class = pcl::RobotEyeGrabber;
    py::class_<Class, pcl::Grabber, boost::shared_ptr<Class>> cls(m, "RobotEyeGrabber");
    cls.def(py::init<>());
    cls.def(py::init<boost::asio::ip::address, unsigned short>(), "ipAddress"_a, "port"_a=443);
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("isRunning", &Class::isRunning);
    cls.def("setSensorAddress", &Class::setSensorAddress, "ipAddress"_a);
    cls.def("setDataPort", &Class::setDataPort, "port"_a);
    cls.def("setSignalPointCloudSize", &Class::setSignalPointCloudSize, "numerOfPoints"_a);
    cls.def("getName", &Class::getName);
    cls.def("getFramesPerSecond", &Class::getFramesPerSecond);
    cls.def("getSensorAddress", &Class::getSensorAddress);
    cls.def("getDataPort", &Class::getDataPort);
    cls.def("getSignalPointCloudSize", &Class::getSignalPointCloudSize);
    cls.def("getPointCloud", &Class::getPointCloud);
}

void defineIoRobotEyeGrabberFunctions(py::module &m) {
}

void defineIoRobotEyeGrabberClasses(py::module &sub_module) {
    defineIoRobotEyeGrabber(sub_module);
}