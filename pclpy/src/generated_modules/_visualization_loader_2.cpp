
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "visualization/point_cloud_geometry_handlers.hpp"
#include "visualization/actor_map.hpp"
#include "visualization/point_picking_event.hpp"
#include "visualization/pcl_visualizer.hpp"
#include "visualization/range_image_visualizer.hpp"


void defineVisualizationClasses2(py::module &m) {
    py::module m_visualization = m.def_submodule("visualization", "Submodule visualization");
    defineVisualizationPointCloudGeometryHandlersClasses(m_visualization);
    defineVisualizationActorMapClasses(m_visualization);
    defineVisualizationPointPickingEventClasses(m_visualization);
    defineVisualizationPclVisualizerClasses(m_visualization);
    defineVisualizationRangeImageVisualizerClasses(m_visualization);
}