
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"
#include "filters/boost.hpp"
#include "filters/clipper3D.hpp"
#include "filters/box_clipper3D.hpp"
#include "filters/convolution.hpp"
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
#include "filters/grid_minimum.hpp"
#include "filters/median_filter.hpp"
#include "filters/morphological_filter.hpp"
#include "filters/normal_refinement.hpp"
#include "filters/normal_space.hpp"
#include "filters/passthrough.hpp"
#include "filters/plane_clipper3D.hpp"
#include "filters/random_sample.hpp"
#include "filters/sampling_surface_normal.hpp"
#include "filters/shadowpoints.hpp"
#include "filters/uniform_sampling.hpp"
#include "filters/voxel_grid.hpp"
#include "filters/voxel_grid_label.hpp"
#include "filters/voxel_grid_occlusion_estimation.hpp"
#include "filters/voxel_grid_covariance.hpp"
#include "filters/project_inliers.hpp"
#include "filters/bilateral.hpp"
#include "filters/convolution_3d.hpp"
#include "filters/local_maximum.hpp"
#include "filters/radius_outlier_removal.hpp"
#include "filters/statistical_outlier_removal.hpp"


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