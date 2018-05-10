
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "filters/project_inliers.hpp"
#include "filters/bilateral.hpp"
#include "filters/local_maximum.hpp"


void defineFiltersClasses8(py::module &m) {
    py::module m_filters = m.def_submodule("filters", "Submodule filters");
    defineFiltersProjectInliersClasses(m_filters);
    defineFiltersBilateralClasses(m_filters);
    defineFiltersLocalMaximumClasses(m_filters);
}