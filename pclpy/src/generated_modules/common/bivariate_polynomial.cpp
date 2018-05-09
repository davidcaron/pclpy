
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/common/bivariate_polynomial.h>



template<typename real>
void defineCommonBivariatePolynomialT(py::module &m, std::string const & suffix) {
    using Class = pcl::BivariatePolynomialT<real>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<int>(), "new_degree"_a=0);
    cls.def_readwrite("degree", &Class::degree);
    cls.def_readwrite("parameters", &Class::parameters);
    cls.def_readwrite("gradient_x", &Class::gradient_x);
    // Operators not implemented (operator=);
    cls.def("calculateGradient", &Class::calculateGradient, "forceRecalc"_a=false);
    cls.def("findCriticalPoints", &Class::findCriticalPoints, "x_values"_a, "y_values"_a, "types"_a);
    cls.def("writeBinary", py::overload_cast<std::ostream &> (&Class::writeBinary, py::const_), "os"_a);
    cls.def("writeBinary", py::overload_cast<const char *> (&Class::writeBinary, py::const_), "filename"_a);
    cls.def("readBinary", py::overload_cast<std::istream &> (&Class::readBinary), "os"_a);
    cls.def("readBinary", py::overload_cast<const char *> (&Class::readBinary), "filename"_a);
    cls.def("setDegree", &Class::setDegree, "new_degree"_a);
    cls.def("getNoOfParameters", &Class::getNoOfParameters);
    cls.def("getValue", &Class::getValue, "x"_a, "y"_a);
    cls.def("getValueOfGradient", &Class::getValueOfGradient, "x"_a, "y"_a, "gradX"_a, "gradY"_a);
    cls.def_static("getNoOfParametersFromDegree", &Class::getNoOfParametersFromDegree, "n"_a);
        
}

void defineCommonBivariatePolynomialFunctions(py::module &m) {
}

void defineCommonBivariatePolynomialClasses(py::module &sub_module) {
    defineCommonBivariatePolynomialFunctions(sub_module);
}