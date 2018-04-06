
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/image_grabber.h>



void defineIoImageGrabberBase(py::module &m) {
    using Class = ImageGrabberBase;
    py::class_<Class, Grabber, boost::shared_ptr<Class>> cls(m, "ImageGrabberBase");
    cls.def("set_rgb_image_files", &Class::setRGBImageFiles);
    cls.def_property("camera_intrinsics", &Class::getCameraIntrinsics, &Class::setCameraIntrinsics);
    cls.def("set_depth_image_units", &Class::setDepthImageUnits);
    cls.def("set_number_of_threads", &Class::setNumberOfThreads);
    // Operators not implemented (operator=);
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("trigger", &Class::trigger);
    cls.def("is_running", &Class::isRunning);
    cls.def("rewind", &Class::rewind);
    cls.def("is_repeat_on", &Class::isRepeatOn);
    cls.def("at_last_frame", &Class::atLastFrame);
}

template <typename PointT>
void defineIoImageGrabber(py::module &m, std::string const & suffix) {
    using Class = ImageGrabber<PointT>;
    py::class_<Class, ImageGrabberBase, FileGrabber<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    // Operators not implemented (operator[]);
    cls.def("size", &Class::size);
        
}

void defineIoImageGrabberClasses(py::module &sub_module) {
    defineIoImageGrabberBase(sub_module);
}