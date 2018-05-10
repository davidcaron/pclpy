
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "surface/vtk_mesh_smoothing_windowed_sinc.hpp"
#include "surface/vtk_mesh_subdivision.hpp"
#include "surface/vtk_utils.hpp"


void defineSurfaceClasses6(py::module &m) {
    py::module m_surface = m.def_submodule("surface", "Submodule surface");
    defineSurfaceVtkMeshSmoothingWindowedSincClasses(m_surface);
    defineSurfaceVtkMeshSubdivisionClasses(m_surface);
    defineSurfaceVtkUtilsClasses(m_surface);
}