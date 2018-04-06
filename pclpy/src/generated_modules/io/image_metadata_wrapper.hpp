
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/io/image_metadata_wrapper.h>

using namespace pcl::io;


void defineIoFrameWrapper(py::module &m) {
    using Class = io::FrameWrapper;
    using Ptr = Class::Ptr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FrameWrapper");
}

void defineIoImageMetadataWrapperClasses(py::module &sub_module) {
    defineIoFrameWrapper(sub_module);
}