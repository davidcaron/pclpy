
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
#include "segmentation/approximate_progressive_morphological_filter.hpp"
#include "segmentation/boost.hpp"
#include "segmentation/comparator.hpp"
#include "segmentation/crf_normal_segmentation.hpp"
#include "segmentation/crf_segmentation.hpp"
#include "segmentation/euclidean_cluster_comparator.hpp"
#include "segmentation/extract_clusters.hpp"
#include "segmentation/extract_labeled_clusters.hpp"
#include "segmentation/extract_polygonal_prism_data.hpp"
#include "segmentation/grabcut_segmentation.hpp"
#include "segmentation/ground_plane_comparator.hpp"
#include "segmentation/min_cut_segmentation.hpp"
#include "segmentation/organized_connected_component_segmentation.hpp"
#include "segmentation/plane_coefficient_comparator.hpp"
#include "segmentation/edge_aware_plane_comparator.hpp"
#include "segmentation/euclidean_plane_coefficient_comparator.hpp"
#include "segmentation/plane_refinement_comparator.hpp"
#include "segmentation/progressive_morphological_filter.hpp"
#include "segmentation/random_walker.hpp"
#include "segmentation/region_3d.hpp"
#include "segmentation/planar_region.hpp"
#include "segmentation/organized_multi_plane_segmentation.hpp"
#include "segmentation/planar_polygon_fusion.hpp"
#include "segmentation/region_growing.hpp"
#include "segmentation/region_growing_rgb.hpp"
#include "segmentation/rgb_plane_coefficient_comparator.hpp"
#include "segmentation/sac_segmentation.hpp"
#include "segmentation/segment_differences.hpp"
#include "segmentation/supervoxel_clustering.hpp"
#include "segmentation/lccp_segmentation.hpp"
#include "segmentation/cpc_segmentation.hpp"
#include "segmentation/unary_classifier.hpp"


void defineSegmentationClasses(py::module &m) {
    py::module m_segmentation = m.def_submodule("segmentation", "Submodule segmentation");
    defineSegmentationApproximateProgressiveMorphologicalFilterClasses(m_segmentation);
    defineSegmentationBoostClasses(m_segmentation);
    defineSegmentationComparatorClasses(m_segmentation);
    defineSegmentationCrfNormalSegmentationClasses(m_segmentation);
    defineSegmentationCrfSegmentationClasses(m_segmentation);
    defineSegmentationEuclideanClusterComparatorClasses(m_segmentation);
    defineSegmentationExtractClustersClasses(m_segmentation);
    defineSegmentationExtractLabeledClustersClasses(m_segmentation);
    defineSegmentationExtractPolygonalPrismDataClasses(m_segmentation);
    defineSegmentationGrabcutSegmentationClasses(m_segmentation);
    defineSegmentationGroundPlaneComparatorClasses(m_segmentation);
    defineSegmentationMinCutSegmentationClasses(m_segmentation);
    defineSegmentationOrganizedConnectedComponentSegmentationClasses(m_segmentation);
    defineSegmentationPlaneCoefficientComparatorClasses(m_segmentation);
    defineSegmentationEdgeAwarePlaneComparatorClasses(m_segmentation);
    defineSegmentationEuclideanPlaneCoefficientComparatorClasses(m_segmentation);
    defineSegmentationPlaneRefinementComparatorClasses(m_segmentation);
    defineSegmentationProgressiveMorphologicalFilterClasses(m_segmentation);
    defineSegmentationRandomWalkerClasses(m_segmentation);
    defineSegmentationRegion3dClasses(m_segmentation);
    defineSegmentationPlanarRegionClasses(m_segmentation);
    defineSegmentationOrganizedMultiPlaneSegmentationClasses(m_segmentation);
    defineSegmentationPlanarPolygonFusionClasses(m_segmentation);
    defineSegmentationRegionGrowingClasses(m_segmentation);
    defineSegmentationRegionGrowingRgbClasses(m_segmentation);
    defineSegmentationRgbPlaneCoefficientComparatorClasses(m_segmentation);
    defineSegmentationSacSegmentationClasses(m_segmentation);
    defineSegmentationSegmentDifferencesClasses(m_segmentation);
    defineSegmentationSupervoxelClusteringClasses(m_segmentation);
    defineSegmentationLccpSegmentationClasses(m_segmentation);
    defineSegmentationCpcSegmentationClasses(m_segmentation);
    defineSegmentationUnaryClassifierClasses(m_segmentation);
}