
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"
#include "visualization/area_picking_event.hpp"
#include "visualization/boost.hpp"
#include "visualization/cloud_viewer.hpp"
#include "visualization/eigen.hpp"
#include "visualization/common.hpp"
#include "visualization/float_image_utils.hpp"
#include "visualization/interactor.hpp"
#include "visualization/keyboard_event.hpp"
#include "visualization/mouse_event.hpp"
#include "visualization/pcl_image_canvas_source_2d.hpp"
#include "visualization/image_viewer.hpp"
#include "visualization/pcl_plotter.hpp"
#include "visualization/point_cloud_color_handlers.hpp"
#include "visualization/point_cloud_geometry_handlers.hpp"
#include "visualization/point_cloud_handlers.hpp"
#include "visualization/actor_map.hpp"
#include "visualization/io.hpp"
#include "visualization/point_picking_event.hpp"
#include "visualization/pcl_visualizer.hpp"
#include "visualization/range_image_visualizer.hpp"
#include "visualization/registration_visualizer.hpp"
#include "visualization/shapes.hpp"
#include "visualization/vtk.hpp"
#include "visualization/vtkRenderWindowInteractorFix.hpp"
#include "visualization/vtkVertexBufferObject.hpp"
#include "visualization/vtkVertexBufferObjectMapper.hpp"
#include "visualization/window.hpp"


void defineVisualizationClasses(py::module &m) {
    py::module m_visualization = m.def_submodule("visualization", "Submodule visualization");
    defineVisualizationAreaPickingEventClasses(m_visualization);
    defineVisualizationBoostClasses(m_visualization);
    defineVisualizationCloudViewerClasses(m_visualization);
    defineVisualizationEigenClasses(m_visualization);
    defineVisualizationCommonClasses(m_visualization);
    defineVisualizationFloatImageUtilsClasses(m_visualization);
    defineVisualizationInteractorClasses(m_visualization);
    defineVisualizationKeyboardEventClasses(m_visualization);
    defineVisualizationMouseEventClasses(m_visualization);
    defineVisualizationPclImageCanvasSource2dClasses(m_visualization);
    defineVisualizationImageViewerClasses(m_visualization);
    defineVisualizationPclPlotterClasses(m_visualization);
    defineVisualizationPointCloudColorHandlersClasses(m_visualization);
    defineVisualizationPointCloudGeometryHandlersClasses(m_visualization);
    defineVisualizationPointCloudHandlersClasses(m_visualization);
    defineVisualizationActorMapClasses(m_visualization);
    defineVisualizationIoClasses(m_visualization);
    defineVisualizationPointPickingEventClasses(m_visualization);
    defineVisualizationPclVisualizerClasses(m_visualization);
    defineVisualizationRangeImageVisualizerClasses(m_visualization);
    defineVisualizationRegistrationVisualizerClasses(m_visualization);
    defineVisualizationShapesClasses(m_visualization);
    defineVisualizationVtkClasses(m_visualization);
    defineVisualizationVtkRenderWindowInteractorFixClasses(m_visualization);
    defineVisualizationVtkVertexBufferObjectClasses(m_visualization);
    defineVisualizationVtkVertexBufferObjectMapperClasses(m_visualization);
    defineVisualizationWindowClasses(m_visualization);
}