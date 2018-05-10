
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
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
#include "filters/filter_indices.hpp"
#include "filters/covariance_sampling.hpp"
#include "filters/crop_box.hpp"
#include "filters/crop_hull.hpp"
#include "filters/extract_indices.hpp"
#include "filters/frustum_culling.hpp"


void defineFiltersClasses(py::module &m) {
    py::module m_filters = m.def_submodule("filters", "Submodule filters");
    defineFiltersFilterClasses(m_filters);
    defineFiltersApproximateVoxelGridClasses(m_filters);
    defineFiltersFastBilateralClasses(m_filters);
    defineFiltersFastBilateralOmpClasses(m_filters);
    defineFiltersFilterIndicesClasses(m_filters);
    defineFiltersCovarianceSamplingClasses(m_filters);
    defineFiltersCropBoxClasses(m_filters);
    defineFiltersCropHullClasses(m_filters);
    defineFiltersExtractIndicesClasses(m_filters);
    defineFiltersFrustumCullingClasses(m_filters);
}