
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "common/piecewise_linear_function.hpp"
#include "common/point_tests.hpp"
#include "common/poses_from_matches.hpp"


void defineCommonClasses4(py::module &m) {
    py::module m_common = m.def_submodule("common", "Submodule common");
    defineCommonPiecewiseLinearFunctionClasses(m_common);
    defineCommonPointTestsClasses(m_common);
    defineCommonPosesFromMatchesClasses(m_common);
}