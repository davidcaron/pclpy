
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../make_opaque_vectors.hpp"

void defineGeometryBoostClasses(py::module &);
void defineGeometryEigenClasses(py::module &);
void defineGeometryGetBoundaryClasses(py::module &);
void defineGeometryMeshConversionClasses(py::module &);
void defineGeometryMeshIndicesClasses(py::module &);
void defineGeometryMeshCirculatorsClasses(py::module &);
void defineGeometryMeshElementsClasses(py::module &);
void defineGeometryMeshIoClasses(py::module &);
void defineGeometryMeshTraitsClasses(py::module &);
void defineGeometryMeshBaseClasses(py::module &);
void defineGeometryOrganizedIndexIteratorClasses(py::module &);
void defineGeometryLineIteratorClasses(py::module &);
void defineGeometryPlanarPolygonClasses(py::module &);
void defineGeometryPolygonMeshClasses(py::module &);
void defineGeometryPolygonOperationsClasses(py::module &);
void defineGeometryQuadMeshClasses(py::module &);
void defineGeometryTriangleMeshClasses(py::module &);


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