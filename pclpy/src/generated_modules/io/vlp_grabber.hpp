
#include <pcl/io/vlp_grabber.h>



void defineIoVLPGrabber(py::module &m) {
    using Class = pcl::VLPGrabber;
    py::class_<Class, pcl::HDLGrabber, boost::shared_ptr<Class>> cls(m, "VLPGrabber");
    cls.def("getName", &Class::getName);
}

void defineIoVlpGrabberFunctions(py::module &m) {
}

void defineIoVlpGrabberClasses(py::module &sub_module) {
    defineIoVLPGrabber(sub_module);
}