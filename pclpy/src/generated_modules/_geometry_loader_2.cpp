
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "geometry/quad_mesh.hpp"
#include "geometry/triangle_mesh.hpp"


void defineGeometryClasses2(py::module &m) {
    py::module m_geometry = m.def_submodule("geometry", "Submodule geometry");
    defineGeometryQuadMeshClasses(m_geometry);
    defineGeometryTriangleMeshClasses(m_geometry);
}