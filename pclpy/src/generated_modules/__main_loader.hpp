
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

void define2dClasses0(py::module &);
void definebaseClasses0(py::module &);
void definebaseClasses1(py::module &);
void definecommonClasses0(py::module &);
void definecommonClasses1(py::module &);
void definecommonClasses2(py::module &);
void definefeaturesClasses0(py::module &);
void definefeaturesClasses1(py::module &);
void definefeaturesClasses2(py::module &);
void definefeaturesClasses3(py::module &);
void definefeaturesClasses4(py::module &);
void definefeaturesClasses5(py::module &);
void definefeaturesClassesfeature(py::module &);
void definefiltersClasses0(py::module &);
void definefiltersClasses1(py::module &);
void definefiltersClasses2(py::module &);
void definefiltersClasses3(py::module &);
void definegeometryClasses0(py::module &);
void definegeometryClasses1(py::module &);
void defineioClasses0(py::module &);
void defineioClasses1(py::module &);
void defineioClasses2(py::module &);
void definekdtreeClasses0(py::module &);
void defineoctreeClasses0(py::module &);
void defineoctreeClasses1(py::module &);
void definerecognitionClasses0(py::module &);
void definerecognitionClasses1(py::module &);
void definerecognitionClasses2(py::module &);
void definesample_consensusClasses0(py::module &);
void definesample_consensusClasses1(py::module &);
void definesample_consensusClasses2(py::module &);
void definesample_consensusClasses3(py::module &);
void definesearchClasses0(py::module &);
void definesegmentationClasses0(py::module &);
void definesegmentationClasses1(py::module &);
void definestereoClasses0(py::module &);
void definesurfaceClasses0(py::module &);
void definesurfaceClasses1(py::module &);
void definesurfaceClasses2(py::module &);
void definetrackingClasses0(py::module &);
void definetrackingClasses1(py::module &);
void definevisualizationClasses0(py::module &);
void definevisualizationClasses1(py::module &);

void defineClasses(py::module &m) {
    define2dClasses0(m);
    definebaseClasses0(m);
    definebaseClasses1(m);
    definecommonClasses0(m);
    definecommonClasses1(m);
    definecommonClasses2(m);
    definefeaturesClasses0(m);
    definefeaturesClasses1(m);
    definefeaturesClasses2(m);
    definefeaturesClasses3(m);
    definefeaturesClasses4(m);
    definefeaturesClasses5(m);
    definefeaturesClassesfeature(m);
    definefiltersClasses0(m);
    definefiltersClasses1(m);
    definefiltersClasses2(m);
    definefiltersClasses3(m);
    definegeometryClasses0(m);
    definegeometryClasses1(m);
    defineioClasses0(m);
    defineioClasses1(m);
    defineioClasses2(m);
    definekdtreeClasses0(m);
    defineoctreeClasses0(m);
    defineoctreeClasses1(m);
    definerecognitionClasses0(m);
    definerecognitionClasses1(m);
    definerecognitionClasses2(m);
    definesample_consensusClasses0(m);
    definesample_consensusClasses1(m);
    definesample_consensusClasses2(m);
    definesample_consensusClasses3(m);
    definesearchClasses0(m);
    definesegmentationClasses0(m);
    definesegmentationClasses1(m);
    definestereoClasses0(m);
    definesurfaceClasses0(m);
    definesurfaceClasses1(m);
    definesurfaceClasses2(m);
    definetrackingClasses0(m);
    definetrackingClasses1(m);
    definevisualizationClasses0(m);
    definevisualizationClasses1(m);
}