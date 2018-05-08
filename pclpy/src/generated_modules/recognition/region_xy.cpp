
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

#include <pcl/recognition/region_xy.h>



void defineRecognitionRegionXY(py::module &m) {
    using Class = pcl::RegionXY;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RegionXY");
    cls.def(py::init<>());
    cls.def_readwrite("x", &Class::x);
    cls.def_readwrite("y", &Class::y);
    cls.def_readwrite("width", &Class::width);
    cls.def_readwrite("height", &Class::height);
    cls.def("serialize", &Class::serialize, "stream"_a);
    cls.def("deserialize", &Class::deserialize, "stream"_a);
}

void defineRecognitionRegionXyFunctions(py::module &m) {
}

void defineRecognitionRegionXyClasses(py::module &sub_module) {
    defineRecognitionRegionXY(sub_module);
    defineRecognitionRegionXyFunctions(sub_module);
}