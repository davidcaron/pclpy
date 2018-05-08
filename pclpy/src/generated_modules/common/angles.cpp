
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

#include <pcl/common/angles.h>



void defineCommonAnglesFunctions1(py::module &m) {
    m.def("deg2rad", py::overload_cast<float> (&pcl::deg2rad), "alpha"_a);
    m.def("deg2rad", py::overload_cast<double> (&pcl::deg2rad), "alpha"_a);
    m.def("normAngle", py::overload_cast<float> (&pcl::normAngle), "alpha"_a);
    m.def("rad2deg", py::overload_cast<float> (&pcl::rad2deg), "alpha"_a);
    m.def("rad2deg", py::overload_cast<double> (&pcl::rad2deg), "alpha"_a);
}

void defineCommonAnglesFunctions(py::module &m) {
    defineCommonAnglesFunctions1(m);
}

void defineCommonAnglesClasses(py::module &sub_module) {
    defineCommonAnglesFunctions(sub_module);
}