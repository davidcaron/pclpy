
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/geometry/mesh_indices.h>

using namespace pcl::geometry;


void defineGeometryEdgeIndex(py::module &m) {
    using Class = pcl::geometry::EdgeIndex;
    using Base = Class::Base;
    using Self = Class::Self;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "EdgeIndex");
    cls.def(py::init<>());
    cls.def(py::init<int>(), "index"_a);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    cls.def("invalidate", &Class::invalidate);
    // Operators not implemented (operator<);
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    // Operators not implemented (operator+=);
    // Operators not implemented (operator-=);
    cls.def("set", &Class::set, "index"_a);
    cls.def("get", &Class::get);
}

void defineGeometryFaceIndex(py::module &m) {
    using Class = pcl::geometry::FaceIndex;
    using Base = Class::Base;
    using Self = Class::Self;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FaceIndex");
    cls.def(py::init<>());
    cls.def(py::init<int>(), "index"_a);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    cls.def("invalidate", &Class::invalidate);
    // Operators not implemented (operator<);
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    // Operators not implemented (operator+=);
    // Operators not implemented (operator-=);
    cls.def("set", &Class::set, "index"_a);
    cls.def("get", &Class::get);
}

void defineGeometryHalfEdgeIndex(py::module &m) {
    using Class = pcl::geometry::HalfEdgeIndex;
    using Base = Class::Base;
    using Self = Class::Self;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "HalfEdgeIndex");
    cls.def(py::init<>());
    cls.def(py::init<int>(), "index"_a);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    cls.def("invalidate", &Class::invalidate);
    // Operators not implemented (operator<);
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    // Operators not implemented (operator+=);
    // Operators not implemented (operator-=);
    cls.def("set", &Class::set, "index"_a);
    cls.def("get", &Class::get);
}

void defineGeometryVertexIndex(py::module &m) {
    using Class = pcl::geometry::VertexIndex;
    using Base = Class::Base;
    using Self = Class::Self;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "VertexIndex");
    cls.def(py::init<>());
    cls.def(py::init<int>(), "index"_a);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    cls.def("invalidate", &Class::invalidate);
    // Operators not implemented (operator<);
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    // Operators not implemented (operator+=);
    // Operators not implemented (operator-=);
    cls.def("set", &Class::set, "index"_a);
    cls.def("get", &Class::get);
}

void defineGeometryMeshIndicesFunctions(py::module &m) {
}

void defineGeometryMeshIndicesClasses(py::module &sub_module) {
    defineGeometryEdgeIndex(sub_module);
    defineGeometryFaceIndex(sub_module);
    defineGeometryHalfEdgeIndex(sub_module);
    defineGeometryVertexIndex(sub_module);
}