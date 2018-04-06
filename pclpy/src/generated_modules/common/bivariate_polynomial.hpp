
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/common/bivariate_polynomial.h>



template<typename real>
void defineCommonBivariatePolynomialT(py::module &m, std::string const & suffix) {
    using Class = BivariatePolynomialT<real>;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<int>(), "new_degree"_a=0);
    cls.def("set_degree", &Class::setDegree);
    cls.def_readonly("degree", &Class::degree);
    cls.def_readonly("parameters", &Class::parameters);
    cls.def_readonly("gradient_x", &Class::gradient_x);
    cls.def("write_binary", py::overload_cast<std::ostream &> (&Class::writeBinary, py::const_));
    cls.def("write_binary", py::overload_cast<const char *> (&Class::writeBinary, py::const_));
    cls.def("read_binary", py::overload_cast<std::istream &> (&Class::readBinary));
    cls.def("read_binary", py::overload_cast<const char *> (&Class::readBinary));
    // Operators not implemented (operator=);
    cls.def("calculate_gradient", &Class::calculateGradient);
    cls.def("find_critical_points", &Class::findCriticalPoints);
        
}

void defineCommonBivariatePolynomialClasses(py::module &sub_module) {
}