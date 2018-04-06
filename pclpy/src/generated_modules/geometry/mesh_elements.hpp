
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/geometry/mesh_elements.h>

using namespace pcl::geometry;


void defineGeometryFace(py::module &m) {
    using Class = geometry::Face;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Face");
}

void defineGeometryHalfEdge(py::module &m) {
    using Class = geometry::HalfEdge;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "HalfEdge");
}

void defineGeometryVertex(py::module &m) {
    using Class = geometry::Vertex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "Vertex");
}

void defineGeometryMeshElementsClasses(py::module &sub_module) {
    defineGeometryFace(sub_module);
    defineGeometryHalfEdge(sub_module);
    defineGeometryVertex(sub_module);
}