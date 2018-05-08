
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

#include <pcl/io/image_ir.h>

using namespace pcl::io;


void defineIoIRImage(py::module &m) {
    using Class = pcl::io::IRImage;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using Clock = Class::Clock;
    using Timestamp = Class::Timestamp;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "IRImage");
    cls.def(py::init<FrameWrapper::Ptr>(), "ir_metadata"_a);
    cls.def(py::init<FrameWrapper::Ptr, Timestamp>(), "ir_metadata"_a, "time"_a);
    cls.def("fillRaw", &Class::fillRaw, "width"_a, "height"_a, "ir_buffer"_a, "line_step"_a=0);
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getFrameID", &Class::getFrameID);
    cls.def("getTimestamp", &Class::getTimestamp);
    cls.def("getSystemTimestamp", &Class::getSystemTimestamp);
    cls.def("getData", &Class::getData);
    cls.def("getDataSize", &Class::getDataSize);
    cls.def("getStep", &Class::getStep);
    cls.def("getMetaData", &Class::getMetaData);
}

void defineIoImageIrFunctions(py::module &m) {
}

void defineIoImageIrClasses(py::module &sub_module) {
    defineIoIRImage(sub_module);
}