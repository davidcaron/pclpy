
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