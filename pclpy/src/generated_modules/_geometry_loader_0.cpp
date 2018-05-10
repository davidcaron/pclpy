
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

#include "geometry/mesh_indices.hpp"
#include "geometry/mesh_elements.hpp"
#include "geometry/mesh_io.hpp"
#include "geometry/mesh_traits.hpp"
#include "geometry/organized_index_iterator.hpp"
#include "geometry/line_iterator.hpp"


void defineGeometryClasses(py::module &m) {
    py::module m_geometry = m.def_submodule("geometry", "Submodule geometry");
    defineGeometryMeshIndicesClasses(m_geometry);
    defineGeometryMeshElementsClasses(m_geometry);
    defineGeometryMeshIoClasses(m_geometry);
    defineGeometryMeshTraitsClasses(m_geometry);
    defineGeometryOrganizedIndexIteratorClasses(m_geometry);
    defineGeometryLineIteratorClasses(m_geometry);
}