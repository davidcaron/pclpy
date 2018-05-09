
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineSegmentationApproximateProgressiveMorphologicalFilterClasses(py::module &);
void defineSegmentationBoostClasses(py::module &);
void defineSegmentationComparatorClasses(py::module &);
void defineSegmentationCrfNormalSegmentationClasses(py::module &);
void defineSegmentationCrfSegmentationClasses(py::module &);
void defineSegmentationEuclideanClusterComparatorClasses(py::module &);
void defineSegmentationExtractClustersClasses(py::module &);
void defineSegmentationExtractLabeledClustersClasses(py::module &);
void defineSegmentationExtractPolygonalPrismDataClasses(py::module &);
void defineSegmentationGrabcutSegmentationClasses(py::module &);
void defineSegmentationGroundPlaneComparatorClasses(py::module &);
void defineSegmentationMinCutSegmentationClasses(py::module &);
void defineSegmentationOrganizedConnectedComponentSegmentationClasses(py::module &);
void defineSegmentationPlaneCoefficientComparatorClasses(py::module &);
void defineSegmentationEdgeAwarePlaneComparatorClasses(py::module &);
void defineSegmentationEuclideanPlaneCoefficientComparatorClasses(py::module &);
void defineSegmentationPlaneRefinementComparatorClasses(py::module &);
void defineSegmentationProgressiveMorphologicalFilterClasses(py::module &);
void defineSegmentationRandomWalkerClasses(py::module &);
void defineSegmentationRegion3dClasses(py::module &);
void defineSegmentationPlanarRegionClasses(py::module &);
void defineSegmentationOrganizedMultiPlaneSegmentationClasses(py::module &);
void defineSegmentationPlanarPolygonFusionClasses(py::module &);
void defineSegmentationRegionGrowingClasses(py::module &);
void defineSegmentationRegionGrowingRgbClasses(py::module &);
void defineSegmentationRgbPlaneCoefficientComparatorClasses(py::module &);
void defineSegmentationSacSegmentationClasses(py::module &);
void defineSegmentationSegmentDifferencesClasses(py::module &);
void defineSegmentationSupervoxelClusteringClasses(py::module &);
void defineSegmentationLccpSegmentationClasses(py::module &);
void defineSegmentationCpcSegmentationClasses(py::module &);
void defineSegmentationUnaryClassifierClasses(py::module &);


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