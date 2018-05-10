
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "filters/extract_indices.hpp"
#include "filters/frustum_culling.hpp"
#include "filters/grid_minimum.hpp"
#include "filters/median_filter.hpp"


void defineFiltersClasses2(py::module &m) {
    py::module m_filters = m.def_submodule("filters", "Submodule filters");
    defineFiltersExtractIndicesClasses(m_filters);
    defineFiltersFrustumCullingClasses(m_filters);
    defineFiltersGridMinimumClasses(m_filters);
    defineFiltersMedianFilterClasses(m_filters);
}