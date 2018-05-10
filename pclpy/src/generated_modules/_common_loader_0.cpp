
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "common/angles.hpp"
#include "common/bivariate_polynomial.hpp"


void defineCommonClasses0(py::module &m) {
    py::module m_common = m.def_submodule("common", "Submodule common");
    defineCommonAnglesClasses(m_common);
    defineCommonBivariatePolynomialClasses(m_common);
}