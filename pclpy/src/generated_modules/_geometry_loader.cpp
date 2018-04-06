
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"
#include "geometry/boost.hpp"
#include "geometry/eigen.hpp"
#include "geometry/get_boundary.hpp"
#include "geometry/mesh_conversion.hpp"
#include "geometry/mesh_indices.hpp"
#include "geometry/mesh_circulators.hpp"
#include "geometry/mesh_elements.hpp"
#include "geometry/mesh_io.hpp"
#include "geometry/mesh_traits.hpp"
#include "geometry/mesh_base.hpp"
#include "geometry/organized_index_iterator.hpp"
#include "geometry/line_iterator.hpp"
#include "geometry/planar_polygon.hpp"
#include "geometry/polygon_mesh.hpp"
#include "geometry/polygon_operations.hpp"
#include "geometry/quad_mesh.hpp"
#include "geometry/triangle_mesh.hpp"


void defineGeometryClasses(py::module &m) {
    py::module m_geometry = m.def_submodule("geometry", "Submodule geometry");
    defineGeometryBoostClasses(m_geometry);
    defineGeometryEigenClasses(m_geometry);
    defineGeometryGetBoundaryClasses(m_geometry);
    defineGeometryMeshConversionClasses(m_geometry);
    defineGeometryMeshIndicesClasses(m_geometry);
    defineGeometryMeshCirculatorsClasses(m_geometry);
    defineGeometryMeshElementsClasses(m_geometry);
    defineGeometryMeshIoClasses(m_geometry);
    defineGeometryMeshTraitsClasses(m_geometry);
    defineGeometryMeshBaseClasses(m_geometry);
    defineGeometryOrganizedIndexIteratorClasses(m_geometry);
    defineGeometryLineIteratorClasses(m_geometry);
    defineGeometryPlanarPolygonClasses(m_geometry);
    defineGeometryPolygonMeshClasses(m_geometry);
    defineGeometryPolygonOperationsClasses(m_geometry);
    defineGeometryQuadMeshClasses(m_geometry);
    defineGeometryTriangleMeshClasses(m_geometry);
}