
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/io/pcd_grabber.h>



void defineIoPCDGrabberBase(py::module &m) {
    using Class = pcl::PCDGrabberBase;
    py::class_<Class, pcl::Grabber, boost::shared_ptr<Class>> cls(m, "PCDGrabberBase");
    // Operators not implemented (operator=);
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("trigger", &Class::trigger);
    cls.def("isRunning", &Class::isRunning);
    cls.def("rewind", &Class::rewind);
    cls.def("isRepeatOn", &Class::isRepeatOn);
    cls.def("numFrames", &Class::numFrames);
    cls.def("getName", &Class::getName);
    cls.def("getFramesPerSecond", &Class::getFramesPerSecond);
    cls.def("getCloudAt", &Class::getCloudAt, "idx"_a, "blob"_a, "origin"_a, "orientation"_a);
}

template <typename PointT>
void defineIoPCDGrabber(py::module &m, std::string const & suffix) {
    using Class = pcl::PCDGrabber<PointT>;
    py::class_<Class, pcl::PCDGrabberBase, pcl::FileGrabber<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    // Operators not implemented (operator[]);
    cls.def("size", &Class::size);
    cls.def("publish", &Class::publish, "blob"_a, "origin"_a, "orientation"_a, "file_name"_a);
        
}

void defineIoPcdGrabberFunctions(py::module &m) {
}

void defineIoPcdGrabberClasses(py::module &sub_module) {
    defineIoPCDGrabberBase(sub_module);
    defineIoPcdGrabberFunctions(sub_module);
}