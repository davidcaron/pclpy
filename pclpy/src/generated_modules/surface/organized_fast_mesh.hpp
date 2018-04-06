
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/organized_fast_mesh.h>



template <typename PointInT>
void defineSurfaceOrganizedFastMesh(py::module &m, std::string const & suffix) {
    using Class = OrganizedFastMesh<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using Polygons = Class::Polygons;
    py::class_<Class, MeshConstruction<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::TriangulationType>(cls, "triangulation_type")
        .value("TRIANGLE_RIGHT_CUT", Class::TriangulationType::TRIANGLE_RIGHT_CUT)
        .value("TRIANGLE_LEFT_CUT", Class::TriangulationType::TRIANGLE_LEFT_CUT)
        .value("TRIANGLE_ADAPTIVE_CUT", Class::TriangulationType::TRIANGLE_ADAPTIVE_CUT)
        .value("QUAD_MESH", Class::TriangulationType::QUAD_MESH)
        .export_values();
    cls.def("set_max_edge_length", &Class::setMaxEdgeLength);
    cls.def("set_triangle_pixel_size", &Class::setTrianglePixelSize);
    cls.def("set_triangle_pixel_size_rows", &Class::setTrianglePixelSizeRows);
    cls.def("set_triangle_pixel_size_columns", &Class::setTrianglePixelSizeColumns);
    cls.def("set_triangulation_type", &Class::setTriangulationType);
    cls.def_property("viewpoint", &Class::getViewpoint, &Class::setViewpoint);
    cls.def("set_angle_tolerance", &Class::setAngleTolerance);
    cls.def("set_distance_tolerance", &Class::setDistanceTolerance);
    cls.def("unset_max_edge_length", &Class::unsetMaxEdgeLength);
    cls.def("store_shadowed_faces", &Class::storeShadowedFaces);
    cls.def("use_depth_as_distance", &Class::useDepthAsDistance);
        
}

void defineSurfaceOrganizedFastMeshClasses(py::module &sub_module) {
    py::module sub_module_OrganizedFastMesh = sub_module.def_submodule("OrganizedFastMesh", "Submodule OrganizedFastMesh");
    defineSurfaceOrganizedFastMesh<PointXYZ>(sub_module_OrganizedFastMesh, "PointXYZ");
    defineSurfaceOrganizedFastMesh<PointXYZRGB>(sub_module_OrganizedFastMesh, "PointXYZRGB");
    defineSurfaceOrganizedFastMesh<PointXYZRGBA>(sub_module_OrganizedFastMesh, "PointXYZRGBA");
}