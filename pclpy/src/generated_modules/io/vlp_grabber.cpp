
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

#include <pcl/io/vlp_grabber.h>



void defineIoVLPGrabber(py::module &m) {
    using Class = pcl::VLPGrabber;
    py::class_<Class, pcl::HDLGrabber, boost::shared_ptr<Class>> cls(m, "VLPGrabber");
    cls.def("getName", &Class::getName);
}

void defineIoVlpGrabberFunctions(py::module &m) {
}

void defineIoVlpGrabberClasses(py::module &sub_module) {
    defineIoVLPGrabber(sub_module);
}