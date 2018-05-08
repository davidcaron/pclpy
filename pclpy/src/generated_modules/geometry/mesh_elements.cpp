
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/geometry/mesh_elements.h>

using namespace pcl::geometry;


void defineGeometryFace(py::module &m) {
    using Class = pcl::geometry::Face;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Face");
}

void defineGeometryHalfEdge(py::module &m) {
    using Class = pcl::geometry::HalfEdge;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "HalfEdge");
}

void defineGeometryVertex(py::module &m) {
    using Class = pcl::geometry::Vertex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Vertex");
}

void defineGeometryMeshElementsFunctions(py::module &m) {
}

void defineGeometryMeshElementsClasses(py::module &sub_module) {
    defineGeometryFace(sub_module);
    defineGeometryHalfEdge(sub_module);
    defineGeometryVertex(sub_module);
}