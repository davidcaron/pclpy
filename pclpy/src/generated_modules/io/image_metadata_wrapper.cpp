
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

#include <pcl/io/image_metadata_wrapper.h>

using namespace pcl::io;


void defineIoFrameWrapper(py::module &m) {
    using Class = pcl::io::FrameWrapper;
    using Ptr = Class::Ptr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FrameWrapper");
    cls.def("getData", &Class::getData);
    cls.def("getDataSize", &Class::getDataSize);
    cls.def("getWidth", &Class::getWidth);
    cls.def("getHeight", &Class::getHeight);
    cls.def("getFrameID", &Class::getFrameID);
    cls.def("getTimestamp", &Class::getTimestamp);
}

void defineIoImageMetadataWrapperFunctions(py::module &m) {
}

void defineIoImageMetadataWrapperClasses(py::module &sub_module) {
    defineIoFrameWrapper(sub_module);
}