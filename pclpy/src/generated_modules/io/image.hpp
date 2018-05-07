
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/io/image.h>

using namespace pcl::io;


void defineIoImage(py::module &m) {
    using Class = pcl::io::Image;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using Clock = Class::Clock;
    using Timestamp = Class::Timestamp;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Image");
    py::enum_<Class::Encoding>(cls, "Encoding")
        .value("BAYER_GRBG", Class::Encoding::BAYER_GRBG)
        .value("YUV422", Class::Encoding::YUV422)
        .value("RGB", Class::Encoding::RGB)
        .export_values();
    cls.def("isResizingSupported", &Class::isResizingSupported, "input_width"_a, "input_height"_a, "output_width"_a, "output_height"_a);
    cls.def("fillRGB", &Class::fillRGB, "width"_a, "height"_a, "rgb_buffer"_a, "rgb_line_step"_a=0);
    cls.def("fillRaw", &Class::fillRaw, "rgb_buffer"_a);
    cls.def("fillGrayscale", &Class::fillGrayscale, "width"_a, "height"_a, "gray_buffer"_a, "gray_line_step"_a=0);
    cls.def("getEncoding", &Class::getEncoding);
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getFrameID", &Class::getFrameID);
    cls.def("getTimestamp", &Class::getTimestamp);
    cls.def("getSystemTimestamp", &Class::getSystemTimestamp);
    cls.def("getData", &Class::getData);
    cls.def("getDataSize", &Class::getDataSize);
    cls.def("getStep", &Class::getStep);
}

void defineIoImageFunctions(py::module &m) {
}

void defineIoImageClasses(py::module &sub_module) {
    defineIoImage(sub_module);
}