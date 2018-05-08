
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineFiltersBoostClasses(py::module &);
void defineFiltersClipper3DClasses(py::module &);
void defineFiltersBoxClipper3DClasses(py::module &);
void defineFiltersConvolutionClasses(py::module &);
void defineFiltersFilterClasses(py::module &);
void defineFiltersApproximateVoxelGridClasses(py::module &);
void defineFiltersFastBilateralClasses(py::module &);
void defineFiltersFastBilateralOmpClasses(py::module &);
void defineFiltersFilterIndicesClasses(py::module &);
void defineFiltersCovarianceSamplingClasses(py::module &);
void defineFiltersCropBoxClasses(py::module &);
void defineFiltersCropHullClasses(py::module &);
void defineFiltersExtractIndicesClasses(py::module &);
void defineFiltersFrustumCullingClasses(py::module &);
void defineFiltersGridMinimumClasses(py::module &);
void defineFiltersMedianFilterClasses(py::module &);
void defineFiltersMorphologicalFilterClasses(py::module &);
void defineFiltersNormalRefinementClasses(py::module &);
void defineFiltersNormalSpaceClasses(py::module &);
void defineFiltersPassthroughClasses(py::module &);
void defineFiltersPlaneClipper3DClasses(py::module &);
void defineFiltersRandomSampleClasses(py::module &);
void defineFiltersSamplingSurfaceNormalClasses(py::module &);
void defineFiltersShadowpointsClasses(py::module &);
void defineFiltersUniformSamplingClasses(py::module &);
void defineFiltersVoxelGridClasses(py::module &);
void defineFiltersVoxelGridLabelClasses(py::module &);
void defineFiltersVoxelGridOcclusionEstimationClasses(py::module &);
void defineFiltersVoxelGridCovarianceClasses(py::module &);
void defineFiltersProjectInliersClasses(py::module &);
void defineFiltersBilateralClasses(py::module &);
void defineFiltersConvolution3dClasses(py::module &);
void defineFiltersLocalMaximumClasses(py::module &);
void defineFiltersRadiusOutlierRemovalClasses(py::module &);
void defineFiltersStatisticalOutlierRemovalClasses(py::module &);


void defineFiltersClasses(py::module &m) {
    py::module m_filters = m.def_submodule("filters", "Submodule filters");
    defineFiltersBoostClasses(m_filters);
    defineFiltersClipper3DClasses(m_filters);
    defineFiltersBoxClipper3DClasses(m_filters);
    defineFiltersConvolutionClasses(m_filters);
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
    defineFiltersGridMinimumClasses(m_filters);
    defineFiltersMedianFilterClasses(m_filters);
    defineFiltersMorphologicalFilterClasses(m_filters);
    defineFiltersNormalRefinementClasses(m_filters);
    defineFiltersNormalSpaceClasses(m_filters);
    defineFiltersPassthroughClasses(m_filters);
    defineFiltersPlaneClipper3DClasses(m_filters);
    defineFiltersRandomSampleClasses(m_filters);
    defineFiltersSamplingSurfaceNormalClasses(m_filters);
    defineFiltersShadowpointsClasses(m_filters);
    defineFiltersUniformSamplingClasses(m_filters);
    defineFiltersVoxelGridClasses(m_filters);
    defineFiltersVoxelGridLabelClasses(m_filters);
    defineFiltersVoxelGridOcclusionEstimationClasses(m_filters);
    defineFiltersVoxelGridCovarianceClasses(m_filters);
    defineFiltersProjectInliersClasses(m_filters);
    defineFiltersBilateralClasses(m_filters);
    defineFiltersConvolution3dClasses(m_filters);
    defineFiltersLocalMaximumClasses(m_filters);
    defineFiltersRadiusOutlierRemovalClasses(m_filters);
    defineFiltersStatisticalOutlierRemovalClasses(m_filters);
}