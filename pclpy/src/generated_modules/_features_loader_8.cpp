
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "features/intensity_spin.hpp"
#include "features/linear_least_squares_normal.hpp"


void defineFeaturesClasses8(py::module &m) {
    py::module m_features = m.def_submodule("features", "Submodule features");
    defineFeaturesIntensitySpinClasses(m_features);
    defineFeaturesLinearLeastSquaresNormalClasses(m_features);
}