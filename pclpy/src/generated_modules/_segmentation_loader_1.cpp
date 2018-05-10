
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "segmentation/min_cut_segmentation.hpp"
#include "segmentation/organized_connected_component_segmentation.hpp"
#include "segmentation/progressive_morphological_filter.hpp"
#include "segmentation/organized_multi_plane_segmentation.hpp"
#include "segmentation/planar_polygon_fusion.hpp"


void defineSegmentationClasses1(py::module &m) {
    py::module m_segmentation = m.def_submodule("segmentation", "Submodule segmentation");
    defineSegmentationMinCutSegmentationClasses(m_segmentation);
    defineSegmentationOrganizedConnectedComponentSegmentationClasses(m_segmentation);
    defineSegmentationProgressiveMorphologicalFilterClasses(m_segmentation);
    defineSegmentationOrganizedMultiPlaneSegmentationClasses(m_segmentation);
    defineSegmentationPlanarPolygonFusionClasses(m_segmentation);
}