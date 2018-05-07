
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/geometry/mesh_base.h>

using namespace pcl::geometry;


template <class DerivedT, class MeshTraitsT, class MeshTagT>
void defineGeometryMeshBase(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::MeshBase<DerivedT, MeshTraitsT, MeshTagT>;
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
    cls.def("addVertex", &Class::addVertex, "vertex_data"_a=VertexData());
    cls.def("addFace", py::overload_cast<const VertexIndices &, const FaceData &, const EdgeData &, const HalfEdgeData &> (&Class::addFace), "vertices"_a, "face_data"_a=FaceData(), "edge_data"_a=EdgeData(), "half_edge_data"_a=HalfEdgeData());
    cls.def("deleteVertex", &Class::deleteVertex, "idx_vertex"_a);
    cls.def("deleteEdge", py::overload_cast<const HalfEdgeIndex &> (&Class::deleteEdge), "idx_he"_a);
    cls.def("deleteEdge", py::overload_cast<const EdgeIndex &> (&Class::deleteEdge), "idx_edge"_a);
    cls.def("deleteFace", py::overload_cast<const FaceIndex &> (&Class::deleteFace), "idx_face"_a);
    cls.def("cleanUp", &Class::cleanUp);
    cls.def("isEqualTopology", &Class::isEqualTopology, "other"_a);
    cls.def("isValid", py::overload_cast<const VertexIndex &> (&Class::isValid, py::const_), "idx_vertex"_a);
    cls.def("isValid", py::overload_cast<const HalfEdgeIndex &> (&Class::isValid, py::const_), "idx_he"_a);
    cls.def("isValid", py::overload_cast<const EdgeIndex &> (&Class::isValid, py::const_), "idx_edge"_a);
    cls.def("isValid", py::overload_cast<const FaceIndex &> (&Class::isValid, py::const_), "idx_face"_a);
    cls.def("isDeleted", py::overload_cast<const VertexIndex &> (&Class::isDeleted, py::const_), "idx_vertex"_a);
    cls.def("isDeleted", py::overload_cast<const HalfEdgeIndex &> (&Class::isDeleted, py::const_), "idx_he"_a);
    cls.def("isDeleted", py::overload_cast<const EdgeIndex &> (&Class::isDeleted, py::const_), "idx_edge"_a);
    cls.def("isDeleted", py::overload_cast<const FaceIndex &> (&Class::isDeleted, py::const_), "idx_face"_a);
    cls.def("isIsolated", &Class::isIsolated, "idx_vertex"_a);
    cls.def("isBoundary", py::overload_cast<const VertexIndex &> (&Class::isBoundary, py::const_), "idx_vertex"_a);
    cls.def("isBoundary", py::overload_cast<const HalfEdgeIndex &> (&Class::isBoundary, py::const_), "idx_he"_a);
    cls.def("isBoundary", py::overload_cast<const EdgeIndex &> (&Class::isBoundary, py::const_), "idx_edge"_a);
    cls.def("isBoundary", py::overload_cast<const FaceIndex &> (&Class::isBoundary, py::const_), "idx_face"_a);
    cls.def("isBoundary", py::overload_cast<const FaceIndex &> (&Class::isBoundary, py::const_), "idx_face"_a);
    cls.def("isManifold", py::overload_cast<const VertexIndex &> (&Class::isManifold, py::const_), "idx_vertex"_a);
    cls.def("isManifold", py::overload_cast<> (&Class::isManifold, py::const_));
    cls.def("sizeVertices", &Class::sizeVertices);
    cls.def("sizeHalfEdges", &Class::sizeHalfEdges);
    cls.def("sizeEdges", &Class::sizeEdges);
    cls.def("sizeFaces", &Class::sizeFaces);
    cls.def("empty", &Class::empty);
    cls.def("emptyVertices", &Class::emptyVertices);
    cls.def("emptyEdges", &Class::emptyEdges);
    cls.def("emptyFaces", &Class::emptyFaces);
    cls.def("reserveVertices", &Class::reserveVertices, "n"_a);
    cls.def("reserveEdges", &Class::reserveEdges, "n"_a);
    cls.def("reserveFaces", &Class::reserveFaces, "n"_a);
    cls.def("resizeVertices", &Class::resizeVertices, "n"_a, "data"_a=VertexData());
    cls.def("resizeEdges", &Class::resizeEdges, "n"_a, "edge_data"_a=EdgeData(), "he_data"_a=HalfEdgeData());
    cls.def("resizeFaces", &Class::resizeFaces, "n"_a, "data"_a=FaceData());
    cls.def("clear", &Class::clear);
    cls.def("setVertexDataCloud", &Class::setVertexDataCloud, "vertex_data_cloud"_a);
    cls.def("setHalfEdgeDataCloud", &Class::setHalfEdgeDataCloud, "half_edge_data_cloud"_a);
    cls.def("setEdgeDataCloud", &Class::setEdgeDataCloud, "edge_data_cloud"_a);
    cls.def("setFaceDataCloud", &Class::setFaceDataCloud, "face_data_cloud"_a);
    cls.def("getOutgoingHalfEdgeIndex", &Class::getOutgoingHalfEdgeIndex, "idx_vertex"_a);
    cls.def("getIncomingHalfEdgeIndex", &Class::getIncomingHalfEdgeIndex, "idx_vertex"_a);
    cls.def("getTerminatingVertexIndex", &Class::getTerminatingVertexIndex, "idx_half_edge"_a);
    cls.def("getOriginatingVertexIndex", &Class::getOriginatingVertexIndex, "idx_half_edge"_a);
    cls.def("getOppositeHalfEdgeIndex", &Class::getOppositeHalfEdgeIndex, "idx_half_edge"_a);
    cls.def("getNextHalfEdgeIndex", &Class::getNextHalfEdgeIndex, "idx_half_edge"_a);
    cls.def("getPrevHalfEdgeIndex", &Class::getPrevHalfEdgeIndex, "idx_half_edge"_a);
    cls.def("getFaceIndex", py::overload_cast<const HalfEdgeIndex &> (&Class::getFaceIndex, py::const_), "idx_half_edge"_a);
    cls.def("getOppositeFaceIndex", &Class::getOppositeFaceIndex, "idx_half_edge"_a);
    cls.def("getInnerHalfEdgeIndex", &Class::getInnerHalfEdgeIndex, "idx_face"_a);
    cls.def("getOuterHalfEdgeIndex", &Class::getOuterHalfEdgeIndex, "idx_face"_a);
    cls.def("getVertexAroundVertexCirculator", py::overload_cast<const VertexIndex &> (&Class::getVertexAroundVertexCirculator, py::const_), "idx_vertex"_a);
    cls.def("getVertexAroundVertexCirculator", py::overload_cast<const HalfEdgeIndex &> (&Class::getVertexAroundVertexCirculator, py::const_), "idx_outgoing_half_edge"_a);
    cls.def("getOutgoingHalfEdgeAroundVertexCirculator", py::overload_cast<const VertexIndex &> (&Class::getOutgoingHalfEdgeAroundVertexCirculator, py::const_), "idx_vertex"_a);
    cls.def("getOutgoingHalfEdgeAroundVertexCirculator", py::overload_cast<const HalfEdgeIndex &> (&Class::getOutgoingHalfEdgeAroundVertexCirculator, py::const_), "idx_outgoing_half_edge"_a);
    cls.def("getIncomingHalfEdgeAroundVertexCirculator", py::overload_cast<const VertexIndex &> (&Class::getIncomingHalfEdgeAroundVertexCirculator, py::const_), "idx_vertex"_a);
    cls.def("getIncomingHalfEdgeAroundVertexCirculator", py::overload_cast<const HalfEdgeIndex &> (&Class::getIncomingHalfEdgeAroundVertexCirculator, py::const_), "idx_incoming_half_edge"_a);
    cls.def("getFaceAroundVertexCirculator", py::overload_cast<const VertexIndex &> (&Class::getFaceAroundVertexCirculator, py::const_), "idx_vertex"_a);
    cls.def("getFaceAroundVertexCirculator", py::overload_cast<const HalfEdgeIndex &> (&Class::getFaceAroundVertexCirculator, py::const_), "idx_outgoing_half_edge"_a);
    cls.def("getVertexAroundFaceCirculator", py::overload_cast<const FaceIndex &> (&Class::getVertexAroundFaceCirculator, py::const_), "idx_face"_a);
    cls.def("getVertexAroundFaceCirculator", py::overload_cast<const HalfEdgeIndex &> (&Class::getVertexAroundFaceCirculator, py::const_), "idx_inner_half_edge"_a);
    cls.def("getInnerHalfEdgeAroundFaceCirculator", py::overload_cast<const FaceIndex &> (&Class::getInnerHalfEdgeAroundFaceCirculator, py::const_), "idx_face"_a);
    cls.def("getInnerHalfEdgeAroundFaceCirculator", py::overload_cast<const HalfEdgeIndex &> (&Class::getInnerHalfEdgeAroundFaceCirculator, py::const_), "idx_inner_half_edge"_a);
    cls.def("getOuterHalfEdgeAroundFaceCirculator", py::overload_cast<const FaceIndex &> (&Class::getOuterHalfEdgeAroundFaceCirculator, py::const_), "idx_face"_a);
    cls.def("getOuterHalfEdgeAroundFaceCirculator", py::overload_cast<const HalfEdgeIndex &> (&Class::getOuterHalfEdgeAroundFaceCirculator, py::const_), "idx_inner_half_edge"_a);
    cls.def("getFaceAroundFaceCirculator", py::overload_cast<const FaceIndex &> (&Class::getFaceAroundFaceCirculator, py::const_), "idx_face"_a);
    cls.def("getFaceAroundFaceCirculator", py::overload_cast<const HalfEdgeIndex &> (&Class::getFaceAroundFaceCirculator, py::const_), "idx_inner_half_edge"_a);
    cls.def("getVertexDataCloud", py::overload_cast<> (&Class::getVertexDataCloud));
    cls.def("getVertexDataCloud", py::overload_cast<> (&Class::getVertexDataCloud, py::const_));
    cls.def("getHalfEdgeDataCloud", py::overload_cast<> (&Class::getHalfEdgeDataCloud));
    cls.def("getHalfEdgeDataCloud", py::overload_cast<> (&Class::getHalfEdgeDataCloud, py::const_));
    cls.def("getEdgeDataCloud", py::overload_cast<> (&Class::getEdgeDataCloud));
    cls.def("getEdgeDataCloud", py::overload_cast<> (&Class::getEdgeDataCloud, py::const_));
    cls.def("getFaceDataCloud", py::overload_cast<> (&Class::getFaceDataCloud));
    cls.def("getFaceDataCloud", py::overload_cast<> (&Class::getFaceDataCloud, py::const_));
    cls.def("getVertexIndex", &Class::getVertexIndex, "vertex_data"_a);
    cls.def("getHalfEdgeIndex", &Class::getHalfEdgeIndex, "half_edge_data"_a);
    cls.def("getEdgeIndex", &Class::getEdgeIndex, "edge_data"_a);
    cls.def("getFaceIndex", py::overload_cast<const FaceData &> (&Class::getFaceIndex, py::const_), "face_data"_a);
        
}

void defineGeometryMeshBaseFunctions(py::module &m) {
}

void defineGeometryMeshBaseClasses(py::module &sub_module) {
}