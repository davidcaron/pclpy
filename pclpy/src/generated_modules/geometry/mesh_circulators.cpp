
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

#include <pcl/geometry/mesh_circulators.h>

using namespace pcl::geometry;


template <class MeshT>
void defineGeometryFaceAroundFaceCirculator(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::FaceAroundFaceCirculator<MeshT>;
    using Base = Class::Base;
    using Self = Class::Self;
    using Mesh = Class::Mesh;
    using FaceIndex = Class::FaceIndex;
    using HalfEdgeIndex = Class::HalfEdgeIndex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<FaceIndex, Mesh*>(), "idx_face"_a, "mesh"_a);
    cls.def(py::init<HalfEdgeIndex, Mesh*>(), "idx_inner_half_edge"_a, "mesh"_a);
    cls.def_readwrite("mesh_", &Class::mesh_);
    cls.def_readwrite("idx_inner_half_edge_", &Class::idx_inner_half_edge_);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    cls.def("getTargetIndex", &Class::getTargetIndex);
    cls.def("getCurrentHalfEdgeIndex", &Class::getCurrentHalfEdgeIndex);
        
}

template <class MeshT>
void defineGeometryFaceAroundVertexCirculator(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::FaceAroundVertexCirculator<MeshT>;
    using Base = Class::Base;
    using Self = Class::Self;
    using Mesh = Class::Mesh;
    using FaceIndex = Class::FaceIndex;
    using VertexIndex = Class::VertexIndex;
    using HalfEdgeIndex = Class::HalfEdgeIndex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<VertexIndex, Mesh*>(), "idx_vertex"_a, "mesh"_a);
    cls.def(py::init<HalfEdgeIndex, Mesh*>(), "idx_outgoing_half_edge"_a, "mesh"_a);
    cls.def_readwrite("mesh_", &Class::mesh_);
    cls.def_readwrite("idx_outgoing_half_edge_", &Class::idx_outgoing_half_edge_);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    cls.def("getTargetIndex", &Class::getTargetIndex);
    cls.def("getCurrentHalfEdgeIndex", &Class::getCurrentHalfEdgeIndex);
        
}

template <class MeshT>
void defineGeometryIncomingHalfEdgeAroundVertexCirculator(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::IncomingHalfEdgeAroundVertexCirculator<MeshT>;
    using Base = Class::Base;
    using Self = Class::Self;
    using Mesh = Class::Mesh;
    using VertexIndex = Class::VertexIndex;
    using HalfEdgeIndex = Class::HalfEdgeIndex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<VertexIndex, Mesh*>(), "idx_vertex"_a, "mesh"_a);
    cls.def(py::init<HalfEdgeIndex, Mesh*>(), "idx_incoming_half_edge"_a, "mesh"_a);
    cls.def_readwrite("mesh_", &Class::mesh_);
    cls.def_readwrite("idx_incoming_half_edge_", &Class::idx_incoming_half_edge_);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    cls.def("getTargetIndex", &Class::getTargetIndex);
    cls.def("getCurrentHalfEdgeIndex", &Class::getCurrentHalfEdgeIndex);
        
}

template <class MeshT>
void defineGeometryInnerHalfEdgeAroundFaceCirculator(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::InnerHalfEdgeAroundFaceCirculator<MeshT>;
    using Base = Class::Base;
    using Self = Class::Self;
    using Mesh = Class::Mesh;
    using FaceIndex = Class::FaceIndex;
    using HalfEdgeIndex = Class::HalfEdgeIndex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<FaceIndex, Mesh*>(), "idx_face"_a, "mesh"_a);
    cls.def(py::init<HalfEdgeIndex, Mesh*>(), "idx_inner_half_edge"_a, "mesh"_a);
    cls.def_readwrite("mesh_", &Class::mesh_);
    cls.def_readwrite("idx_inner_half_edge_", &Class::idx_inner_half_edge_);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    cls.def("getTargetIndex", &Class::getTargetIndex);
    cls.def("getCurrentHalfEdgeIndex", &Class::getCurrentHalfEdgeIndex);
        
}

template <class MeshT>
void defineGeometryOuterHalfEdgeAroundFaceCirculator(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::OuterHalfEdgeAroundFaceCirculator<MeshT>;
    using Base = Class::Base;
    using Self = Class::Self;
    using Mesh = Class::Mesh;
    using FaceIndex = Class::FaceIndex;
    using HalfEdgeIndex = Class::HalfEdgeIndex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<FaceIndex, Mesh*>(), "idx_face"_a, "mesh"_a);
    cls.def(py::init<HalfEdgeIndex, Mesh*>(), "idx_inner_half_edge"_a, "mesh"_a);
    cls.def_readwrite("mesh_", &Class::mesh_);
    cls.def_readwrite("idx_inner_half_edge_", &Class::idx_inner_half_edge_);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    cls.def("getTargetIndex", &Class::getTargetIndex);
    cls.def("getCurrentHalfEdgeIndex", &Class::getCurrentHalfEdgeIndex);
        
}

template <class MeshT>
void defineGeometryOutgoingHalfEdgeAroundVertexCirculator(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::OutgoingHalfEdgeAroundVertexCirculator<MeshT>;
    using Base = Class::Base;
    using Self = Class::Self;
    using Mesh = Class::Mesh;
    using VertexIndex = Class::VertexIndex;
    using HalfEdgeIndex = Class::HalfEdgeIndex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<VertexIndex, Mesh*>(), "idx_vertex"_a, "mesh"_a);
    cls.def(py::init<HalfEdgeIndex, Mesh*>(), "idx_outgoing_half_edge"_a, "mesh"_a);
    cls.def_readwrite("mesh_", &Class::mesh_);
    cls.def_readwrite("idx_outgoing_half_edge_", &Class::idx_outgoing_half_edge_);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    cls.def("getTargetIndex", &Class::getTargetIndex);
    cls.def("getCurrentHalfEdgeIndex", &Class::getCurrentHalfEdgeIndex);
        
}

template <class MeshT>
void defineGeometryVertexAroundFaceCirculator(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::VertexAroundFaceCirculator<MeshT>;
    using Base = Class::Base;
    using Self = Class::Self;
    using Mesh = Class::Mesh;
    using VertexIndex = Class::VertexIndex;
    using FaceIndex = Class::FaceIndex;
    using HalfEdgeIndex = Class::HalfEdgeIndex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<FaceIndex, Mesh*>(), "idx_face"_a, "mesh"_a);
    cls.def(py::init<HalfEdgeIndex, Mesh*>(), "idx_inner_half_edge"_a, "mesh"_a);
    cls.def_readwrite("mesh_", &Class::mesh_);
    cls.def_readwrite("idx_inner_half_edge_", &Class::idx_inner_half_edge_);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    cls.def("getTargetIndex", &Class::getTargetIndex);
    cls.def("getCurrentHalfEdgeIndex", &Class::getCurrentHalfEdgeIndex);
        
}

template <class MeshT>
void defineGeometryVertexAroundVertexCirculator(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::VertexAroundVertexCirculator<MeshT>;
    using Base = Class::Base;
    using Self = Class::Self;
    using Mesh = Class::Mesh;
    using VertexIndex = Class::VertexIndex;
    using HalfEdgeIndex = Class::HalfEdgeIndex;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<VertexIndex, Mesh*>(), "idx_vertex"_a, "mesh"_a);
    cls.def(py::init<HalfEdgeIndex, Mesh*>(), "idx_outgoing_half_edge"_a, "mesh"_a);
    cls.def_readwrite("mesh_", &Class::mesh_);
    cls.def_readwrite("idx_outgoing_half_edge_", &Class::idx_outgoing_half_edge_);
    cls.def("isValid", py::overload_cast<> (&Class::isValid, py::const_));
    // Operators not implemented (operator==);
    // Operators not implemented (operator++);
    // Operators not implemented (operator--);
    cls.def("getTargetIndex", &Class::getTargetIndex);
    cls.def("getCurrentHalfEdgeIndex", &Class::getCurrentHalfEdgeIndex);
        
}

void defineGeometryMeshCirculatorsFunctions(py::module &m) {
}

void defineGeometryMeshCirculatorsClasses(py::module &sub_module) {
}