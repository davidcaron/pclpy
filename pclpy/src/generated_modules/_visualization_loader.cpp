
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

void defineVisualizationAreaPickingEventClasses(py::module &);
void defineVisualizationBoostClasses(py::module &);
void defineVisualizationCloudViewerClasses(py::module &);
void defineVisualizationEigenClasses(py::module &);
void defineVisualizationCommonClasses(py::module &);
void defineVisualizationFloatImageUtilsClasses(py::module &);
void defineVisualizationInteractorClasses(py::module &);
void defineVisualizationKeyboardEventClasses(py::module &);
void defineVisualizationMouseEventClasses(py::module &);
void defineVisualizationPclImageCanvasSource2dClasses(py::module &);
void defineVisualizationImageViewerClasses(py::module &);
void defineVisualizationPclPlotterClasses(py::module &);
void defineVisualizationPointCloudColorHandlersClasses(py::module &);
void defineVisualizationPointCloudGeometryHandlersClasses(py::module &);
void defineVisualizationPointCloudHandlersClasses(py::module &);
void defineVisualizationActorMapClasses(py::module &);
void defineVisualizationIoClasses(py::module &);
void defineVisualizationPointPickingEventClasses(py::module &);
void defineVisualizationPclVisualizerClasses(py::module &);
void defineVisualizationRangeImageVisualizerClasses(py::module &);
void defineVisualizationRegistrationVisualizerClasses(py::module &);
void defineVisualizationShapesClasses(py::module &);
void defineVisualizationVtkClasses(py::module &);
void defineVisualizationVtkRenderWindowInteractorFixClasses(py::module &);
void defineVisualizationVtkVertexBufferObjectClasses(py::module &);
void defineVisualizationVtkVertexBufferObjectMapperClasses(py::module &);
void defineVisualizationWindowClasses(py::module &);


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