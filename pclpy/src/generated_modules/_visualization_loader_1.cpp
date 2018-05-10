
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "visualization/mouse_event.hpp"
#include "visualization/pcl_image_canvas_source_2d.hpp"
#include "visualization/image_viewer.hpp"
#include "visualization/pcl_plotter.hpp"
#include "visualization/point_cloud_color_handlers.hpp"


void defineVisualizationClasses1(py::module &m) {
    py::module m_visualization = m.def_submodule("visualization", "Submodule visualization");
    defineVisualizationMouseEventClasses(m_visualization);
    defineVisualizationPclImageCanvasSource2dClasses(m_visualization);
    defineVisualizationImageViewerClasses(m_visualization);
    defineVisualizationPclPlotterClasses(m_visualization);
    defineVisualizationPointCloudColorHandlersClasses(m_visualization);
}