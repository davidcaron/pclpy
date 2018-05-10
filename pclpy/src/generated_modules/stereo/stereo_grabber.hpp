
#include <pcl/stereo/stereo_grabber.h>



void defineStereoStereoGrabberBase(py::module &m) {
    using Class = pcl::StereoGrabberBase;
    py::class_<Class, pcl::Grabber, boost::shared_ptr<Class>> cls(m, "StereoGrabberBase");
    // Operators not implemented (operator=);
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("trigger", &Class::trigger);
    cls.def("isRunning", &Class::isRunning);
    cls.def("rewind", &Class::rewind);
    cls.def("isRepeatOn", &Class::isRepeatOn);
    cls.def("getName", &Class::getName);
    cls.def("getFramesPerSecond", &Class::getFramesPerSecond);
}

template <typename PointT>
void defineStereoStereoGrabber(py::module &m, std::string const & suffix) {
    using Class = pcl::StereoGrabber<PointT>;
    py::class_<Class, pcl::StereoGrabberBase, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("publish", &Class::publish, "blob"_a, "origin"_a, "orientation"_a);
        
}

void defineStereoStereoGrabberFunctions(py::module &m) {
}

void defineStereoStereoGrabberClasses(py::module &sub_module) {
    defineStereoStereoGrabberBase(sub_module);
    defineStereoStereoGrabberFunctions(sub_module);
}