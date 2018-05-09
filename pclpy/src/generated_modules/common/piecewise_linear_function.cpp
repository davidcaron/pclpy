
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/common/piecewise_linear_function.h>



void defineCommonPiecewiseLinearFunction(py::module &m) {
    using Class = pcl::PiecewiseLinearFunction;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PiecewiseLinearFunction");
    cls.def(py::init<float, float>(), "factor"_a, "offset"_a);
    cls.def("getDataPoints", &Class::getDataPoints);
    cls.def("getValue", &Class::getValue, "point"_a);
}

void defineCommonPiecewiseLinearFunctionFunctions(py::module &m) {
}

void defineCommonPiecewiseLinearFunctionClasses(py::module &sub_module) {
    defineCommonPiecewiseLinearFunction(sub_module);
}