
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

#include <pcl/surface/organized_fast_mesh.h>



template <typename PointInT>
void defineSurfaceOrganizedFastMesh(py::module &m, std::string const & suffix) {
    using Class = pcl::OrganizedFastMesh<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using Polygons = Class::Polygons;
    py::class_<Class, pcl::MeshConstruction<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::TriangulationType>(cls, "TriangulationType")
        .value("TRIANGLE_RIGHT_CUT", Class::TriangulationType::TRIANGLE_RIGHT_CUT)
        .value("TRIANGLE_LEFT_CUT", Class::TriangulationType::TRIANGLE_LEFT_CUT)
        .value("TRIANGLE_ADAPTIVE_CUT", Class::TriangulationType::TRIANGLE_ADAPTIVE_CUT)
        .value("QUAD_MESH", Class::TriangulationType::QUAD_MESH)
        .export_values();
    cls.def("unsetMaxEdgeLength", &Class::unsetMaxEdgeLength);
    cls.def("storeShadowedFaces", &Class::storeShadowedFaces, "enable"_a);
    cls.def("useDepthAsDistance", &Class::useDepthAsDistance, "enable"_a);
    cls.def("setMaxEdgeLength", &Class::setMaxEdgeLength, "a"_a, "b"_a=0.0f, "c"_a=0.0f);
    cls.def("setTrianglePixelSize", &Class::setTrianglePixelSize, "triangle_size"_a);
    cls.def("setTrianglePixelSizeRows", &Class::setTrianglePixelSizeRows, "triangle_size"_a);
    cls.def("setTrianglePixelSizeColumns", &Class::setTrianglePixelSizeColumns, "triangle_size"_a);
    cls.def("setTriangulationType", &Class::setTriangulationType, "type"_a);
    cls.def("setViewpoint", &Class::setViewpoint, "viewpoint"_a);
    cls.def("setAngleTolerance", &Class::setAngleTolerance, "angle_tolerance"_a);
    cls.def("setDistanceTolerance", &Class::setDistanceTolerance, "distance_tolerance"_a, "depth_dependent"_a=false);
    cls.def("getViewpoint", &Class::getViewpoint);
        
}

void defineSurfaceOrganizedFastMeshFunctions(py::module &m) {
}

void defineSurfaceOrganizedFastMeshClasses(py::module &sub_module) {
    py::module sub_module_OrganizedFastMesh = sub_module.def_submodule("OrganizedFastMesh", "Submodule OrganizedFastMesh");
    defineSurfaceOrganizedFastMesh<pcl::PointXYZ>(sub_module_OrganizedFastMesh, "PointXYZ");
    defineSurfaceOrganizedFastMesh<pcl::PointXYZRGB>(sub_module_OrganizedFastMesh, "PointXYZRGB");
    defineSurfaceOrganizedFastMesh<pcl::PointXYZRGBA>(sub_module_OrganizedFastMesh, "PointXYZRGBA");
}