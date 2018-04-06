
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/geometry/mesh_base.h>

using namespace pcl::geometry;


template <class DerivedT, class MeshTraitsT, class MeshTagT>
void defineGeometryMeshBase(py::module &m, std::string const & suffix) {
    using Class = geometry::MeshBase<DerivedT, MeshTraitsT, MeshTagT>;
    using Self = Class::Self;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using Derived = Class::Derived;
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
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_vertex_data_cloud", &Class::setVertexDataCloud);
    cls.def("set_half_edge_data_cloud", &Class::setHalfEdgeDataCloud);
    cls.def("set_edge_data_cloud", &Class::setEdgeDataCloud);
    cls.def("set_face_data_cloud", &Class::setFaceDataCloud);
    cls.def("add_face", py::overload_cast<const VertexIndices &, const FaceData &, const EdgeData &, const HalfEdgeData &> (&Class::addFace));
    cls.def("delete_edge", py::overload_cast<const HalfEdgeIndex &> (&Class::deleteEdge));
    cls.def("delete_edge", py::overload_cast<const EdgeIndex &> (&Class::deleteEdge));
    cls.def("delete_face", py::overload_cast<const FaceIndex &> (&Class::deleteFace));
    cls.def("is_valid", py::overload_cast<const VertexIndex &> (&Class::isValid, py::const_));
    cls.def("is_valid", py::overload_cast<const HalfEdgeIndex &> (&Class::isValid, py::const_));
    cls.def("is_valid", py::overload_cast<const EdgeIndex &> (&Class::isValid, py::const_));
    cls.def("is_valid", py::overload_cast<const FaceIndex &> (&Class::isValid, py::const_));
    cls.def("is_deleted", py::overload_cast<const VertexIndex &> (&Class::isDeleted, py::const_));
    cls.def("is_deleted", py::overload_cast<const HalfEdgeIndex &> (&Class::isDeleted, py::const_));
    cls.def("is_deleted", py::overload_cast<const EdgeIndex &> (&Class::isDeleted, py::const_));
    cls.def("is_deleted", py::overload_cast<const FaceIndex &> (&Class::isDeleted, py::const_));
    cls.def("is_boundary", py::overload_cast<const VertexIndex &> (&Class::isBoundary, py::const_));
    cls.def("is_boundary", py::overload_cast<const HalfEdgeIndex &> (&Class::isBoundary, py::const_));
    cls.def("is_boundary", py::overload_cast<const EdgeIndex &> (&Class::isBoundary, py::const_));
    cls.def("is_boundary", py::overload_cast<const FaceIndex &> (&Class::isBoundary, py::const_));
    cls.def("is_manifold", py::overload_cast<const VertexIndex &> (&Class::isManifold, py::const_));
    cls.def("is_manifold", py::overload_cast<> (&Class::isManifold, py::const_));
    cls.def("add_vertex", &Class::addVertex);
    cls.def("delete_vertex", &Class::deleteVertex);
    cls.def("clean_up", &Class::cleanUp);
    cls.def("is_equal_topology", &Class::isEqualTopology);
    cls.def("is_isolated", &Class::isIsolated);
    cls.def("size_vertices", &Class::sizeVertices);
    cls.def("size_half_edges", &Class::sizeHalfEdges);
    cls.def("size_edges", &Class::sizeEdges);
    cls.def("size_faces", &Class::sizeFaces);
    cls.def("empty", &Class::empty);
    cls.def("empty_vertices", &Class::emptyVertices);
    cls.def("empty_edges", &Class::emptyEdges);
    cls.def("empty_faces", &Class::emptyFaces);
    cls.def("reserve_vertices", &Class::reserveVertices);
    cls.def("reserve_edges", &Class::reserveEdges);
    cls.def("reserve_faces", &Class::reserveFaces);
    cls.def("resize_vertices", &Class::resizeVertices);
    cls.def("resize_edges", &Class::resizeEdges);
    cls.def("resize_faces", &Class::resizeFaces);
    cls.def("clear", &Class::clear);
    cls.def("get_face_index", py::overload_cast<const HalfEdgeIndex &> (&Class::getFaceIndex, py::const_));
    cls.def("get_vertex_around_vertex_circulator", py::overload_cast<const VertexIndex &> (&Class::getVertexAroundVertexCirculator, py::const_));
    cls.def("get_vertex_around_vertex_circulator", py::overload_cast<const HalfEdgeIndex &> (&Class::getVertexAroundVertexCirculator, py::const_));
    cls.def("get_outgoing_half_edge_around_vertex_circulator", py::overload_cast<const VertexIndex &> (&Class::getOutgoingHalfEdgeAroundVertexCirculator, py::const_));
    cls.def("get_outgoing_half_edge_around_vertex_circulator", py::overload_cast<const HalfEdgeIndex &> (&Class::getOutgoingHalfEdgeAroundVertexCirculator, py::const_));
    cls.def("get_incoming_half_edge_around_vertex_circulator", py::overload_cast<const VertexIndex &> (&Class::getIncomingHalfEdgeAroundVertexCirculator, py::const_));
    cls.def("get_incoming_half_edge_around_vertex_circulator", py::overload_cast<const HalfEdgeIndex &> (&Class::getIncomingHalfEdgeAroundVertexCirculator, py::const_));
    cls.def("get_face_around_vertex_circulator", py::overload_cast<const VertexIndex &> (&Class::getFaceAroundVertexCirculator, py::const_));
    cls.def("get_face_around_vertex_circulator", py::overload_cast<const HalfEdgeIndex &> (&Class::getFaceAroundVertexCirculator, py::const_));
    cls.def("get_vertex_around_face_circulator", py::overload_cast<const FaceIndex &> (&Class::getVertexAroundFaceCirculator, py::const_));
    cls.def("get_vertex_around_face_circulator", py::overload_cast<const HalfEdgeIndex &> (&Class::getVertexAroundFaceCirculator, py::const_));
    cls.def("get_inner_half_edge_around_face_circulator", py::overload_cast<const FaceIndex &> (&Class::getInnerHalfEdgeAroundFaceCirculator, py::const_));
    cls.def("get_inner_half_edge_around_face_circulator", py::overload_cast<const HalfEdgeIndex &> (&Class::getInnerHalfEdgeAroundFaceCirculator, py::const_));
    cls.def("get_outer_half_edge_around_face_circulator", py::overload_cast<const FaceIndex &> (&Class::getOuterHalfEdgeAroundFaceCirculator, py::const_));
    cls.def("get_outer_half_edge_around_face_circulator", py::overload_cast<const HalfEdgeIndex &> (&Class::getOuterHalfEdgeAroundFaceCirculator, py::const_));
    cls.def("get_face_around_face_circulator", py::overload_cast<const FaceIndex &> (&Class::getFaceAroundFaceCirculator, py::const_));
    cls.def("get_face_around_face_circulator", py::overload_cast<const HalfEdgeIndex &> (&Class::getFaceAroundFaceCirculator, py::const_));
    cls.def("get_vertex_data_cloud", py::overload_cast<> (&Class::getVertexDataCloud));
    cls.def("get_vertex_data_cloud", py::overload_cast<> (&Class::getVertexDataCloud, py::const_));
    cls.def("get_half_edge_data_cloud", py::overload_cast<> (&Class::getHalfEdgeDataCloud));
    cls.def("get_half_edge_data_cloud", py::overload_cast<> (&Class::getHalfEdgeDataCloud, py::const_));
    cls.def("get_edge_data_cloud", py::overload_cast<> (&Class::getEdgeDataCloud));
    cls.def("get_edge_data_cloud", py::overload_cast<> (&Class::getEdgeDataCloud, py::const_));
    cls.def("get_face_data_cloud", py::overload_cast<> (&Class::getFaceDataCloud));
    cls.def("get_face_data_cloud", py::overload_cast<> (&Class::getFaceDataCloud, py::const_));
    cls.def("get_face_index", py::overload_cast<const FaceData &> (&Class::getFaceIndex, py::const_));
        
}

void defineGeometryMeshBaseClasses(py::module &sub_module) {
}