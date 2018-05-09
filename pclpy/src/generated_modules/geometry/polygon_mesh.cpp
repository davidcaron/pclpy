
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/geometry/polygon_mesh.h>

using namespace pcl::geometry;


template <class MeshTraitsT>
void defineGeometryPolygonMesh(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::PolygonMesh<MeshTraitsT>;
    using Base = Class::Base;
    using Self = Class::Self;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using VertexData = Class::VertexData;
    using HalfEdgeData = Class::HalfEdgeData;
    using EdgeData = Class::EdgeData;
    using FaceData = Class::FaceData;
    using IsManifold = Class::IsManifold;
    using MeshTag = Class::MeshTag;
    using HasVertexData = Class::HasVertexData;
    using HasHalfEdgeData = Class::HasHalfEdgeData;
    using HasEdgeData = Class::HasEdgeData;
    using HasFaceData = Class::HasFaceData;
    using VertexDataCloud = Class::VertexDataCloud;
    using HalfEdgeDataCloud = Class::HalfEdgeDataCloud;
    using EdgeDataCloud = Class::EdgeDataCloud;
    using FaceDataCloud = Class::FaceDataCloud;
    using VertexIndex = Class::VertexIndex;
    using HalfEdgeIndex = Class::HalfEdgeIndex;
    using EdgeIndex = Class::EdgeIndex;
    using FaceIndex = Class::FaceIndex;
    using VertexIndices = Class::VertexIndices;
    using HalfEdgeIndices = Class::HalfEdgeIndices;
    using EdgeIndices = Class::EdgeIndices;
    using FaceIndices = Class::FaceIndices;
    using VertexAroundVertexCirculator = Class::VertexAroundVertexCirculator;
    using OutgoingHalfEdgeAroundVertexCirculator = Class::OutgoingHalfEdgeAroundVertexCirculator;
    using IncomingHalfEdgeAroundVertexCirculator = Class::IncomingHalfEdgeAroundVertexCirculator;
    using FaceAroundVertexCirculator = Class::FaceAroundVertexCirculator;
    using VertexAroundFaceCirculator = Class::VertexAroundFaceCirculator;
    using InnerHalfEdgeAroundFaceCirculator = Class::InnerHalfEdgeAroundFaceCirculator;
    using OuterHalfEdgeAroundFaceCirculator = Class::OuterHalfEdgeAroundFaceCirculator;
    using FaceAroundFaceCirculator = Class::FaceAroundFaceCirculator;
    py::class_<Class, pcl::geometry::MeshBase<pcl::geometry::PolygonMesh<pcl::geometry::MeshTraitsT>, pcl::geometry::MeshTraitsT, pcl::geometry::PolygonMeshTag>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("addFace", py::overload_cast<const VertexIndex &, const VertexIndex &, const VertexIndex &, const FaceData &, const EdgeData &, const HalfEdgeData &> (&Class::addFace), "idx_v_0"_a, "idx_v_1"_a, "idx_v_2"_a, "face_data"_a=FaceData(), "edge_data"_a=EdgeData(), "half_edge_data"_a=HalfEdgeData());
    cls.def("addFace", py::overload_cast<const VertexIndex &, const VertexIndex &, const VertexIndex &, const VertexIndex &, const FaceData &, const EdgeData &, const HalfEdgeData &> (&Class::addFace), "idx_v_0"_a, "idx_v_1"_a, "idx_v_2"_a, "idx_v_3"_a, "face_data"_a=FaceData(), "edge_data"_a=EdgeData(), "half_edge_data"_a=HalfEdgeData());
        
}

void defineGeometryPolygonMeshTag(py::module &m) {
    using Class = pcl::geometry::PolygonMeshTag;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "PolygonMeshTag");
}

void defineGeometryPolygonMeshFunctions(py::module &m) {
}

void defineGeometryPolygonMeshClasses(py::module &sub_module) {
    defineGeometryPolygonMeshTag(sub_module);
}