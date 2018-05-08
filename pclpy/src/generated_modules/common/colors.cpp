
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

#include <pcl/common/colors.h>



void defineCommonGlasbeyLUT(py::module &m) {
    using Class = pcl::GlasbeyLUT;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "GlasbeyLUT");
    cls.def_static("at", &Class::at, "color_id"_a);
    cls.def_static("size", &Class::size);
    cls.def_static("data", &Class::data);
}

void defineCommonColorsFunctions1(py::module &m) {
    m.def("getRandomColor", py::overload_cast<double, double> (&pcl::getRandomColor), "min"_a=0.2, "max"_a=2.8);
}

void defineCommonColorsFunctions(py::module &m) {
    defineCommonColorsFunctions1(m);
}

void defineCommonColorsClasses(py::module &sub_module) {
    defineCommonGlasbeyLUT(sub_module);
    defineCommonColorsFunctions(sub_module);
}