
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/surface/on_nurbs/triangulation.h>



void defineSurfaceTriangulationFunctions(py::module &m) {
}

void defineSurfaceTriangulationClasses(py::module &sub_module) {
}