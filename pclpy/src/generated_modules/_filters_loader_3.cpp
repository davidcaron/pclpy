
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "filters/morphological_filter.hpp"
#include "filters/normal_refinement.hpp"
#include "filters/passthrough.hpp"


void defineFiltersClasses3(py::module &m) {
    py::module m_filters = m.def_submodule("filters", "Submodule filters");
    defineFiltersMorphologicalFilterClasses(m_filters);
    defineFiltersNormalRefinementClasses(m_filters);
    defineFiltersPassthroughClasses(m_filters);
}