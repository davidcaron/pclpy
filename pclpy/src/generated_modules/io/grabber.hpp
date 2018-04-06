
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/grabber.h>



void defineIoGrabber(py::module &m) {
    using Class = Grabber;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Grabber");
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("is_running", &Class::isRunning);
}

void defineIoGrabberClasses(py::module &sub_module) {
    defineIoGrabber(sub_module);
}