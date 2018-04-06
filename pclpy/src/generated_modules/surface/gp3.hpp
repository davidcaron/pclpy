
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/gp3.h>



template <typename PointInT>
void defineSurfaceGreedyProjectionTriangulation(py::module &m, std::string const & suffix) {
    using Class = GreedyProjectionTriangulation<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, MeshConstruction<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::GP3Type>(cls, "gp3_type")
        .value("NONE", Class::GP3Type::NONE)
        .value("FREE", Class::GP3Type::FREE)
        .value("FRINGE", Class::GP3Type::FRINGE)
        .value("BOUNDARY", Class::GP3Type::BOUNDARY)
        .value("COMPLETED", Class::GP3Type::COMPLETED)
        .export_values();
    cls.def_property("mu", &Class::getMu, &Class::setMu);
    cls.def_property("maximum_nearest_neighbors", &Class::getMaximumNearestNeighbors, &Class::setMaximumNearestNeighbors);
    cls.def_property("search_radius", &Class::getSearchRadius, &Class::setSearchRadius);
    cls.def_property("minimum_angle", &Class::getMinimumAngle, &Class::setMinimumAngle);
    cls.def_property("maximum_angle", &Class::getMaximumAngle, &Class::setMaximumAngle);
    cls.def_property("maximum_surface_angle", &Class::getMaximumSurfaceAngle, &Class::setMaximumSurfaceAngle);
    cls.def_property("normal_consistency", &Class::getNormalConsistency, &Class::setNormalConsistency);
    cls.def_property("consistent_vertex_ordering", &Class::getConsistentVertexOrdering, &Class::setConsistentVertexOrdering);
        
}

void defineSurfaceGp3Classes(py::module &sub_module) {
    py::module sub_module_GreedyProjectionTriangulation = sub_module.def_submodule("GreedyProjectionTriangulation", "Submodule GreedyProjectionTriangulation");
    defineSurfaceGreedyProjectionTriangulation<PointNormal>(sub_module_GreedyProjectionTriangulation, "PointNormal");
    defineSurfaceGreedyProjectionTriangulation<PointXYZINormal>(sub_module_GreedyProjectionTriangulation, "PointXYZINormal");
    defineSurfaceGreedyProjectionTriangulation<PointXYZRGBNormal>(sub_module_GreedyProjectionTriangulation, "PointXYZRGBNormal");
}