
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/geometry/mesh_indices.h>

using namespace pcl::geometry;


void defineGeometryEdgeIndex(py::module &m) {
    using Class = geometry::EdgeIndex;
    using Base = Class::Base;
    using Self = Class::Self;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "EdgeIndex");
    cls.def(py::init<>());
    cls.def(py::init<int>(), "index"_a);
    cls.def_property("", &Class::get, &Class::set);
    cls.def("is_valid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator++);
    cls.def("invalidate", &Class::invalidate);
    // Operators not implemented (operator<);
    // Operators not implemented (operator==);
    // Operators not implemented (operator--);
    // Operators not implemented (operator+=);
    // Operators not implemented (operator-=);
}

void defineGeometryFaceIndex(py::module &m) {
    using Class = geometry::FaceIndex;
    using Base = Class::Base;
    using Self = Class::Self;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "FaceIndex");
    cls.def(py::init<>());
    cls.def(py::init<int>(), "index"_a);
    cls.def_property("", &Class::get, &Class::set);
    cls.def("is_valid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator++);
    cls.def("invalidate", &Class::invalidate);
    // Operators not implemented (operator<);
    // Operators not implemented (operator==);
    // Operators not implemented (operator--);
    // Operators not implemented (operator+=);
    // Operators not implemented (operator-=);
}

void defineGeometryHalfEdgeIndex(py::module &m) {
    using Class = geometry::HalfEdgeIndex;
    using Base = Class::Base;
    using Self = Class::Self;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "HalfEdgeIndex");
    cls.def(py::init<>());
    cls.def(py::init<int>(), "index"_a);
    cls.def_property("", &Class::get, &Class::set);
    cls.def("is_valid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator++);
    cls.def("invalidate", &Class::invalidate);
    // Operators not implemented (operator<);
    // Operators not implemented (operator==);
    // Operators not implemented (operator--);
    // Operators not implemented (operator+=);
    // Operators not implemented (operator-=);
}

void defineGeometryVertexIndex(py::module &m) {
    using Class = geometry::VertexIndex;
    using Base = Class::Base;
    using Self = Class::Self;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "VertexIndex");
    cls.def(py::init<>());
    cls.def(py::init<int>(), "index"_a);
    cls.def_property("", &Class::get, &Class::set);
    cls.def("is_valid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator++);
    cls.def("invalidate", &Class::invalidate);
    // Operators not implemented (operator<);
    // Operators not implemented (operator==);
    // Operators not implemented (operator--);
    // Operators not implemented (operator+=);
    // Operators not implemented (operator-=);
}

void defineGeometryMeshIndicesClasses(py::module &sub_module) {
    defineGeometryEdgeIndex(sub_module);
    defineGeometryFaceIndex(sub_module);
    defineGeometryHalfEdgeIndex(sub_module);
    defineGeometryVertexIndex(sub_module);
}