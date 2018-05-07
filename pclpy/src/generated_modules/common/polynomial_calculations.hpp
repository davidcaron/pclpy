
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/common/polynomial_calculations.h>



template <typename real>
void defineCommonPolynomialCalculationsT(py::module &m, std::string const & suffix) {
    using Class = pcl::PolynomialCalculationsT<real>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("solveQuarticEquation", &Class::solveQuarticEquation, "a"_a, "b"_a, "c"_a, "d"_a, "e"_a, "roots"_a);
    cls.def("solveCubicEquation", &Class::solveCubicEquation, "a"_a, "b"_a, "c"_a, "d"_a, "roots"_a);
    cls.def("solveQuadraticEquation", &Class::solveQuadraticEquation, "a"_a, "b"_a, "c"_a, "roots"_a);
    cls.def("solveLinearEquation", &Class::solveLinearEquation, "a"_a, "b"_a, "roots"_a);
    cls.def("bivariatePolynomialApproximation", py::overload_cast<std::vector<Eigen::Matrix<real, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<real, 3, 1> > > &, unsigned int, bool &> (&Class::bivariatePolynomialApproximation, py::const_), "samplePoints"_a, "polynomial_degree"_a, "error"_a);
    cls.def("bivariatePolynomialApproximation", py::overload_cast<std::vector<Eigen::Matrix<real, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<real, 3, 1> > > &, unsigned int, Class::BivariatePolynomialT<real> &> (&Class::bivariatePolynomialApproximation, py::const_), "samplePoints"_a, "polynomial_degree"_a, "ret"_a);
    cls.def("setZeroValue", &Class::setZeroValue, "new_zero_value"_a);
        
}

void defineCommonPolynomialCalculationsFunctions(py::module &m) {
}

void defineCommonPolynomialCalculationsClasses(py::module &sub_module) {
}