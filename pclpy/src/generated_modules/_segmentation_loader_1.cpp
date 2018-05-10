
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "segmentation/organized_multi_plane_segmentation.hpp"
#include "segmentation/planar_polygon_fusion.hpp"
#include "segmentation/region_growing.hpp"
#include "segmentation/sac_segmentation.hpp"
#include "segmentation/segment_differences.hpp"
#include "segmentation/lccp_segmentation.hpp"
#include "segmentation/cpc_segmentation.hpp"
#include "segmentation/unary_classifier.hpp"


void defineSegmentationClasses1(py::module &m) {
    py::module m_segmentation = m.def_submodule("segmentation", "Submodule segmentation");
    defineSegmentationOrganizedMultiPlaneSegmentationClasses(m_segmentation);
    defineSegmentationPlanarPolygonFusionClasses(m_segmentation);
    defineSegmentationRegionGrowingClasses(m_segmentation);
    defineSegmentationSacSegmentationClasses(m_segmentation);
    defineSegmentationSegmentDifferencesClasses(m_segmentation);
    defineSegmentationLccpSegmentationClasses(m_segmentation);
    defineSegmentationCpcSegmentationClasses(m_segmentation);
    defineSegmentationUnaryClassifierClasses(m_segmentation);
}