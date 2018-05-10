
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "visualization/area_picking_event.hpp"
#include "visualization/cloud_viewer.hpp"
#include "visualization/common.hpp"
#include "visualization/float_image_utils.hpp"


void defineVisualizationClasses0(py::module &m) {
    py::module m_visualization = m.def_submodule("visualization", "Submodule visualization");
    defineVisualizationAreaPickingEventClasses(m_visualization);
    defineVisualizationCloudViewerClasses(m_visualization);
    defineVisualizationCommonClasses(m_visualization);
    defineVisualizationFloatImageUtilsClasses(m_visualization);
}