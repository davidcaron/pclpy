
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/geometry/mesh_traits.h>

using namespace pcl::geometry;


template <class VertexDataT   = pcl::geometry::NoData,
              class HalfEdgeDataT = pcl::geometry::NoData,
              class EdgeDataT     = pcl::geometry::NoData,
              class FaceDataT     = pcl::geometry::NoData>
void defineGeometryDefaultMeshTraits(py::module &m, std::string const & suffix) {
    using Class = pcl::geometry::DefaultMeshTraits<VertexDataT, HalfEdgeDataT, EdgeDataT, FaceDataT>;
    using VertexData = Class::VertexData;
    using HalfEdgeData = Class::HalfEdgeData;
    using EdgeData = Class::EdgeData;
    using FaceData = Class::FaceData;
    using IsManifold = Class::IsManifold;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
        
}

void defineGeometryNoData(py::module &m) {
    using Class = pcl::geometry::NoData;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "NoData");
    cls.def(py::init<>());
}

void defineGeometryMeshTraitsFunctions(py::module &m) {
}

void defineGeometryMeshTraitsClasses(py::module &sub_module) {
    defineGeometryNoData(sub_module);
}