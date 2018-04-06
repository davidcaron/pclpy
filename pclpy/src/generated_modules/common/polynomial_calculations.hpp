
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/polynomial_calculations.h>



template <typename real>
void defineCommonPolynomialCalculationsT(py::module &m, std::string const & suffix) {
    using Class = PolynomialCalculationsT<real>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_zero_value", &Class::setZeroValue);
    cls.def("bivariate_polynomial_approximation", py::overload_cast<std::vector<Eigen::Matrix<real, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<real, 3, 1> > > &, unsigned int, bool &> (&Class::bivariatePolynomialApproximation, py::const_));
    cls.def("bivariate_polynomial_approximation", py::overload_cast<std::vector<Eigen::Matrix<real, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<real, 3, 1> > > &, unsigned int, Class::BivariatePolynomialT<real> &> (&Class::bivariatePolynomialApproximation, py::const_));
    cls.def("solve_quartic_equation", &Class::solveQuarticEquation);
    cls.def("solve_cubic_equation", &Class::solveCubicEquation);
    cls.def("solve_quadratic_equation", &Class::solveQuadraticEquation);
    cls.def("solve_linear_equation", &Class::solveLinearEquation);
        
}

void defineCommonPolynomialCalculationsClasses(py::module &sub_module) {
}