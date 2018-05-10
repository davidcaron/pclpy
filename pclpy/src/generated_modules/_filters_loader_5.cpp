
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "filters/bilateral.hpp"
#include "filters/local_maximum.hpp"
#include "filters/radius_outlier_removal.hpp"
#include "filters/statistical_outlier_removal.hpp"


void defineFiltersClasses5(py::module &m) {
    py::module m_filters = m.def_submodule("filters", "Submodule filters");
    defineFiltersBilateralClasses(m_filters);
    defineFiltersLocalMaximumClasses(m_filters);
    defineFiltersRadiusOutlierRemovalClasses(m_filters);
    defineFiltersStatisticalOutlierRemovalClasses(m_filters);
}