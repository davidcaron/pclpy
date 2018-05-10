
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "segmentation/organized_multi_plane_segmentation.hpp"
#include "segmentation/planar_polygon_fusion.hpp"


void defineSegmentationClasses4(py::module &m) {
    py::module m_segmentation = m.def_submodule("segmentation", "Submodule segmentation");
    defineSegmentationOrganizedMultiPlaneSegmentationClasses(m_segmentation);
    defineSegmentationPlanarPolygonFusionClasses(m_segmentation);
}