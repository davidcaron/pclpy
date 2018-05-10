
#include <pybind11/pybind11.h>
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


void defineSegmentationClasses0(py::module &m) {
    py::module m_segmentation = m.def_submodule("segmentation", "Submodule segmentation");
    defineSegmentationApproximateProgressiveMorphologicalFilterClasses(m_segmentation);
    defineSegmentationExtractClustersClasses(m_segmentation);
    defineSegmentationExtractLabeledClustersClasses(m_segmentation);
    defineSegmentationExtractPolygonalPrismDataClasses(m_segmentation);
}