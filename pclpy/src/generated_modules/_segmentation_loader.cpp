
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "segmentation/approximate_progressive_morphological_filter.hpp"
#include "segmentation/extract_clusters.hpp"
#include "segmentation/extract_labeled_clusters.hpp"
#include "segmentation/extract_polygonal_prism_data.hpp"
#include "segmentation/grabcut_segmentation.hpp"
#include "segmentation/min_cut_segmentation.hpp"
#include "segmentation/organized_connected_component_segmentation.hpp"
#include "segmentation/progressive_morphological_filter.hpp"
#include "segmentation/organized_multi_plane_segmentation.hpp"
#include "segmentation/planar_polygon_fusion.hpp"
#include "segmentation/region_growing.hpp"
#include "segmentation/sac_segmentation.hpp"
#include "segmentation/segment_differences.hpp"
#include "segmentation/lccp_segmentation.hpp"
#include "segmentation/cpc_segmentation.hpp"
#include "segmentation/unary_classifier.hpp"


void defineSegmentationClasses(py::module &m) {
    py::module m_segmentation = m.def_submodule("segmentation", "Submodule segmentation");
    defineSegmentationApproximateProgressiveMorphologicalFilterClasses(m_segmentation);
    defineSegmentationExtractClustersClasses(m_segmentation);
    defineSegmentationExtractLabeledClustersClasses(m_segmentation);
    defineSegmentationExtractPolygonalPrismDataClasses(m_segmentation);
    defineSegmentationGrabcutSegmentationClasses(m_segmentation);
    defineSegmentationMinCutSegmentationClasses(m_segmentation);
    defineSegmentationOrganizedConnectedComponentSegmentationClasses(m_segmentation);
    defineSegmentationProgressiveMorphologicalFilterClasses(m_segmentation);
    defineSegmentationOrganizedMultiPlaneSegmentationClasses(m_segmentation);
    defineSegmentationPlanarPolygonFusionClasses(m_segmentation);
    defineSegmentationRegionGrowingClasses(m_segmentation);
    defineSegmentationSacSegmentationClasses(m_segmentation);
    defineSegmentationSegmentDifferencesClasses(m_segmentation);
    defineSegmentationLccpSegmentationClasses(m_segmentation);
    defineSegmentationCpcSegmentationClasses(m_segmentation);
    defineSegmentationUnaryClassifierClasses(m_segmentation);
}