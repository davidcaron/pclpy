
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/pcd_grabber.h>



void defineIoPCDGrabberBase(py::module &m) {
    using Class = PCDGrabberBase;
    py::class_<Class, Grabber, boost::shared_ptr<Class>> cls(m, "PCDGrabberBase");
    // Operators not implemented (operator=);
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("trigger", &Class::trigger);
    cls.def("is_running", &Class::isRunning);
    cls.def("rewind", &Class::rewind);
    cls.def("is_repeat_on", &Class::isRepeatOn);
    cls.def("num_frames", &Class::numFrames);
}

template <typename PointT>
void defineIoPCDGrabber(py::module &m, std::string const & suffix) {
    using Class = PCDGrabber<PointT>;
    py::class_<Class, PCDGrabberBase, FileGrabber<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    // Operators not implemented (operator[]);
    cls.def("size", &Class::size);
        
}

void defineIoPcdGrabberClasses(py::module &sub_module) {
    defineIoPCDGrabberBase(sub_module);
}