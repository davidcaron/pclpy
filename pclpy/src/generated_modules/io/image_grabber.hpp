
#include <pcl/io/image_grabber.h>



void defineIoImageGrabberBase(py::module &m) {
    using Class = pcl::ImageGrabberBase;
    py::class_<Class, pcl::Grabber, boost::shared_ptr<Class>> cls(m, "ImageGrabberBase");
    // Operators not implemented (operator=);
    cls.def("start", &Class::start);
    cls.def("stop", &Class::stop);
    cls.def("trigger", &Class::trigger);
    cls.def("isRunning", &Class::isRunning);
    cls.def("rewind", &Class::rewind);
    cls.def("isRepeatOn", &Class::isRepeatOn);
    cls.def("atLastFrame", &Class::atLastFrame);
    cls.def("setRGBImageFiles", &Class::setRGBImageFiles, "rgb_image_files"_a);
    cls.def("setCameraIntrinsics", &Class::setCameraIntrinsics, "focal_length_x"_a, "focal_length_y"_a, "principal_point_x"_a, "principal_point_y"_a);
    cls.def("setDepthImageUnits", &Class::setDepthImageUnits, "units"_a);
    cls.def("setNumberOfThreads", &Class::setNumberOfThreads, "nr_threads"_a=0);
    cls.def("getName", &Class::getName);
    cls.def("getFramesPerSecond", &Class::getFramesPerSecond);
    cls.def("getCurrentDepthFileName", &Class::getCurrentDepthFileName);
    cls.def("getPrevDepthFileName", &Class::getPrevDepthFileName);
    cls.def("getDepthFileNameAtIndex", &Class::getDepthFileNameAtIndex, "idx"_a);
    cls.def("getTimestampAtIndex", &Class::getTimestampAtIndex, "idx"_a, "timestamp"_a);
    cls.def("getCameraIntrinsics", &Class::getCameraIntrinsics, "focal_length_x"_a, "focal_length_y"_a, "principal_point_x"_a, "principal_point_y"_a);
}

template <typename PointT>
void defineIoImageGrabber(py::module &m, std::string const & suffix) {
    using Class = pcl::ImageGrabber<PointT>;
    py::class_<Class, pcl::ImageGrabberBase, pcl::FileGrabber<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    // Operators not implemented (operator[]);
    cls.def("size", &Class::size);
    cls.def("publish", &Class::publish, "blob"_a, "origin"_a, "orientation"_a);
        
}

void defineIoImageGrabberFunctions(py::module &m) {
}

void defineIoImageGrabberClasses(py::module &sub_module) {
    defineIoImageGrabberBase(sub_module);
    defineIoImageGrabberFunctions(sub_module);
}