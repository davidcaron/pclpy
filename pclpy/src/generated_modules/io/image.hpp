
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/image.h>

using namespace pcl::io;


void defineIoImage(py::module &m) {
    using Class = io::Image;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using Clock = Class::Clock;
    using Timestamp = Class::Timestamp;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Image");
    py::enum_<Class::Encoding>(cls, "encoding")
        .value("BAYER_GRBG", Class::Encoding::BAYER_GRBG)
        .value("YUV422", Class::Encoding::YUV422)
        .value("RGB", Class::Encoding::RGB)
        .export_values();
    cls.def("is_resizing_supported", &Class::isResizingSupported);
    cls.def("fill_rgb", &Class::fillRGB);
    cls.def("fill_raw", &Class::fillRaw);
    cls.def("fill_grayscale", &Class::fillGrayscale);
}

void defineIoImageClasses(py::module &sub_module) {
    defineIoImage(sub_module);
}