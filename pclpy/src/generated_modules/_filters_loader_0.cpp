
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "filters/filter.hpp"
#include "filters/approximate_voxel_grid.hpp"
#include "filters/fast_bilateral.hpp"
#include "filters/fast_bilateral_omp.hpp"


void defineFiltersClasses0(py::module &m) {
    py::module m_filters = m.def_submodule("filters", "Submodule filters");
    defineFiltersFilterClasses(m_filters);
    defineFiltersApproximateVoxelGridClasses(m_filters);
    defineFiltersFastBilateralClasses(m_filters);
    defineFiltersFastBilateralOmpClasses(m_filters);
}