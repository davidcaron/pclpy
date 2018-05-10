
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

void define2dClasses(py::module &);
void defineBaseClasses(py::module &);
void defineCommonClasses(py::module &);
void defineFeaturesClasses(py::module &);
void defineFiltersClasses(py::module &);
void defineGeometryClasses(py::module &);
void defineIoClasses(py::module &);
void defineKdtreeClasses(py::module &);
void defineOctreeClasses(py::module &);
void defineRecognitionClasses(py::module &);
void defineSampleConsensusClasses(py::module &);
void defineSearchClasses(py::module &);
void defineSegmentationClasses(py::module &);
void defineStereoClasses(py::module &);
void defineSurfaceClasses(py::module &);
void defineTrackingClasses(py::module &);
void defineVisualizationClasses(py::module &);

void defineClasses(py::module &m) {
    define2dClasses(m);
    defineBaseClasses(m);
    defineCommonClasses(m);
    defineFeaturesClasses(m);
    defineFiltersClasses(m);
    defineGeometryClasses(m);
    defineIoClasses(m);
    defineKdtreeClasses(m);
    defineOctreeClasses(m);
    defineRecognitionClasses(m);
    defineSampleConsensusClasses(m);
    defineSearchClasses(m);
    defineSegmentationClasses(m);
    defineStereoClasses(m);
    defineSurfaceClasses(m);
    defineTrackingClasses(m);
    defineVisualizationClasses(m);
}