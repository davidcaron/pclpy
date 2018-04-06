
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/stereo/stereo_grabber.h>



void defineStereoStereoGrabberBase(py::module &m) {
    using Class = StereoGrabberBase;
    py::class_<Class, Grabber, boost::shared_ptr<Class>> cls(m, "StereoGrabberBase");
    // Operators not implemented (operator=);
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("trigger", &Class::trigger);
    cls.def("is_running", &Class::isRunning);
    cls.def("rewind", &Class::rewind);
    cls.def("is_repeat_on", &Class::isRepeatOn);
}

template <typename PointT>
void defineStereoStereoGrabber(py::module &m, std::string const & suffix) {
    using Class = StereoGrabber<PointT>;
    py::class_<Class, StereoGrabberBase, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

void defineStereoStereoGrabberClasses(py::module &sub_module) {
    defineStereoStereoGrabberBase(sub_module);
}