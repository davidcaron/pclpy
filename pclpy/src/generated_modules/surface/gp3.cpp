
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/surface/gp3.h>



template <typename PointInT>
void defineSurfaceGreedyProjectionTriangulation(py::module &m, std::string const & suffix) {
    using Class = pcl::GreedyProjectionTriangulation<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using PointCloudInConstPtr = Class::PointCloudInConstPtr;
    py::class_<Class, pcl::MeshConstruction<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    py::enum_<Class::GP3Type>(cls, "GP3Type")
        .value("NONE", Class::GP3Type::NONE)
        .value("FREE", Class::GP3Type::FREE)
        .value("FRINGE", Class::GP3Type::FRINGE)
        .value("BOUNDARY", Class::GP3Type::BOUNDARY)
        .value("COMPLETED", Class::GP3Type::COMPLETED)
        .export_values();
    cls.def("setMu", &Class::setMu, "mu"_a);
    cls.def("setMaximumNearestNeighbors", &Class::setMaximumNearestNeighbors, "nnn"_a);
    cls.def("setSearchRadius", &Class::setSearchRadius, "radius"_a);
    cls.def("setMinimumAngle", &Class::setMinimumAngle, "minimum_angle"_a);
    cls.def("setMaximumAngle", &Class::setMaximumAngle, "maximum_angle"_a);
    cls.def("setMaximumSurfaceAngle", &Class::setMaximumSurfaceAngle, "eps_angle"_a);
    cls.def("setNormalConsistency", &Class::setNormalConsistency, "consistent"_a);
    cls.def("setConsistentVertexOrdering", &Class::setConsistentVertexOrdering, "consistent_ordering"_a);
    cls.def("getMu", &Class::getMu);
    cls.def("getMaximumNearestNeighbors", &Class::getMaximumNearestNeighbors);
    cls.def("getSearchRadius", &Class::getSearchRadius);
    cls.def("getMinimumAngle", &Class::getMinimumAngle);
    cls.def("getMaximumAngle", &Class::getMaximumAngle);
    cls.def("getMaximumSurfaceAngle", &Class::getMaximumSurfaceAngle);
    cls.def("getNormalConsistency", &Class::getNormalConsistency);
    cls.def("getConsistentVertexOrdering", &Class::getConsistentVertexOrdering);
    cls.def("getPointStates", &Class::getPointStates);
    cls.def("getPartIDs", &Class::getPartIDs);
    cls.def("getSFN", &Class::getSFN);
    cls.def("getFFN", &Class::getFFN);
        
}

void defineSurfaceGp3Functions1(py::module &m) {
    m.def("isVisible", py::overload_cast<const Eigen::Vector2f &, const Eigen::Vector2f &, const Eigen::Vector2f &, const Eigen::Vector2f &> (&pcl::isVisible), "X"_a, "S1"_a, "S2"_a, "R"_a=Eigen::Vector2f::Zero());
}

void defineSurfaceGp3Functions(py::module &m) {
    defineSurfaceGp3Functions1(m);
}

void defineSurfaceGp3Classes(py::module &sub_module) {
    py::module sub_module_GreedyProjectionTriangulation = sub_module.def_submodule("GreedyProjectionTriangulation", "Submodule GreedyProjectionTriangulation");
    defineSurfaceGreedyProjectionTriangulation<pcl::PointNormal>(sub_module_GreedyProjectionTriangulation, "PointNormal");
    defineSurfaceGreedyProjectionTriangulation<pcl::PointXYZINormal>(sub_module_GreedyProjectionTriangulation, "PointXYZINormal");
    defineSurfaceGreedyProjectionTriangulation<pcl::PointXYZRGBNormal>(sub_module_GreedyProjectionTriangulation, "PointXYZRGBNormal");
    defineSurfaceGp3Functions(sub_module);
}