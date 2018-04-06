
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/recognition/region_xy.h>



void defineRecognitionRegionXY(py::module &m) {
    using Class = RegionXY;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "RegionXY");
    cls.def(py::init<>());
    cls.def_readonly("x", &Class::x);
    cls.def_readonly("y", &Class::y);
    cls.def_readonly("width", &Class::width);
    cls.def_readonly("height", &Class::height);
    cls.def("serialize", &Class::serialize);
    cls.def("deserialize", &Class::deserialize);
}

void defineRecognitionRegionXyClasses(py::module &sub_module) {
    defineRecognitionRegionXY(sub_module);
}