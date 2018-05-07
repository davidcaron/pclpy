
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/io/image_depth.h>

using namespace pcl::io;


void defineIoDepthImage(py::module &m) {
    using Class = pcl::io::DepthImage;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using Clock = Class::Clock;
    using Timestamp = Class::Timestamp;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "DepthImage");
    cls.def(py::init<FrameWrapper::Ptr, float, float, pcl::uint64_t, pcl::uint64_t>(), "depth_metadata"_a, "baseline"_a, "focal_length"_a, "shadow_value"_a, "no_sample_value"_a);
    cls.def(py::init<FrameWrapper::Ptr, float, float, pcl::uint64_t, pcl::uint64_t, Timestamp>(), "depth_metadata"_a, "baseline"_a, "focal_length"_a, "shadow_value"_a, "no_sample_value"_a, "time"_a);
    cls.def("fillDisparityImage", &Class::fillDisparityImage, "width"_a, "height"_a, "disparity_buffer"_a, "line_step"_a=0);
    cls.def("fillDepthImage", &Class::fillDepthImage, "width"_a, "height"_a, "depth_buffer"_a, "line_step"_a=0);
    cls.def("fillDepthImageRaw", &Class::fillDepthImageRaw, "width"_a, "height"_a, "depth_buffer"_a, "line_step"_a=0);
    cls.def("getMetaData", &Class::getMetaData);
    cls.def("getBaseline", &Class::getBaseline);
    cls.def("getFocalLength", &Class::getFocalLength);
    cls.def("getShadowValue", &Class::getShadowValue);
    cls.def("getNoSampleValue", &Class::getNoSampleValue);
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getFrameID", &Class::getFrameID);
    cls.def("getTimestamp", &Class::getTimestamp);
    cls.def("getSystemTimestamp", &Class::getSystemTimestamp);
    cls.def("getData", &Class::getData);
    cls.def("getDataSize", &Class::getDataSize);
    cls.def("getStep", &Class::getStep);
}

void defineIoImageDepthFunctions(py::module &m) {
}

void defineIoImageDepthClasses(py::module &sub_module) {
    defineIoDepthImage(sub_module);
}