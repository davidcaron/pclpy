
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/surface/simplification_remove_unused_vertices.h>

using namespace pcl::surface;


void defineSurfaceSimplificationRemoveUnusedVertices(py::module &m) {
    using Class = pcl::surface::SimplificationRemoveUnusedVertices;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "SimplificationRemoveUnusedVertices");
    cls.def(py::init<>());
    cls.def("simplify", py::overload_cast<const pcl::PolygonMesh &, pcl::PolygonMesh &> (&Class::simplify), "input"_a, "output"_a);
    cls.def("simplify", py::overload_cast<const pcl::PolygonMesh &, pcl::PolygonMesh &, std::vector<int> &> (&Class::simplify), "input"_a, "output"_a, "indices"_a);
}

void defineSurfaceSimplificationRemoveUnusedVerticesFunctions(py::module &m) {
}

void defineSurfaceSimplificationRemoveUnusedVerticesClasses(py::module &sub_module) {
    defineSurfaceSimplificationRemoveUnusedVertices(sub_module);
}