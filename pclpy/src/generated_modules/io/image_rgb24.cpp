
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/io/image_rgb24.h>

using namespace pcl::io;


void defineIoImageRGB24(py::module &m) {
    using Class = pcl::io::ImageRGB24;
    py::class_<Class, pcl::io::Image, boost::shared_ptr<Class>> cls(m, "ImageRGB24");
    cls.def(py::init<FrameWrapper::Ptr>(), "image_metadata"_a);
    cls.def(py::init<FrameWrapper::Ptr, Class::Timestamp>(), "image_metadata"_a, "timestamp"_a);
    cls.def("fillRGB", &Class::fillRGB, "width"_a, "height"_a, "rgb_buffer"_a, "rgb_line_step"_a=0);
    cls.def("fillGrayscale", &Class::fillGrayscale, "width"_a, "height"_a, "gray_buffer"_a, "gray_line_step"_a=0);
    cls.def("isResizingSupported", &Class::isResizingSupported, "input_width"_a, "input_height"_a, "output_width"_a, "output_height"_a);
    cls.def("getEncoding", &Class::getEncoding);
}

void defineIoImageRgb24Functions(py::module &m) {
}

void defineIoImageRgb24Classes(py::module &sub_module) {
    defineIoImageRGB24(sub_module);
}