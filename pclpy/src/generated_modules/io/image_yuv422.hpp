
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/image_yuv422.h>

using namespace pcl::io;


void defineIoImageYUV422(py::module &m) {
    using Class = io::ImageYUV422;
    py::class_<Class, io::Image, boost::shared_ptr<Class>> cls(m, "ImageYUV422");
    cls.def(py::init<FrameWrapper::Ptr>(), "image_metadata"_a);
    cls.def(py::init<FrameWrapper::Ptr, Class::Timestamp>(), "image_metadata"_a, "timestamp"_a);
    cls.def("fill_rgb", &Class::fillRGB);
    cls.def("fill_grayscale", &Class::fillGrayscale);
    cls.def("is_resizing_supported", &Class::isResizingSupported);
}

void defineIoImageYuv422Classes(py::module &sub_module) {
    defineIoImageYUV422(sub_module);
}