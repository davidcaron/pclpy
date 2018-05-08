
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

#include <pcl/surface/poisson.h>



template<typename PointNT>
void defineSurfacePoisson(py::module &m, std::string const & suffix) {
    using Class = pcl::Poisson<PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    py::class_<Class, pcl::SurfaceReconstruction<PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("performReconstruction", py::overload_cast<pcl::PolygonMesh &> (&Class::performReconstruction), "output"_a);
    cls.def("performReconstruction", py::overload_cast<pcl::PointCloud<PointNT> &, std::vector<pcl::Vertices> &> (&Class::performReconstruction), "points"_a, "polygons"_a);
    cls.def("setDepth", &Class::setDepth, "depth"_a);
    cls.def("setMinDepth", &Class::setMinDepth, "min_depth"_a);
    cls.def("setPointWeight", &Class::setPointWeight, "point_weight"_a);
    cls.def("setScale", &Class::setScale, "scale"_a);
    cls.def("setSolverDivide", &Class::setSolverDivide, "solver_divide"_a);
    cls.def("setIsoDivide", &Class::setIsoDivide, "iso_divide"_a);
    cls.def("setSamplesPerNode", &Class::setSamplesPerNode, "samples_per_node"_a);
    cls.def("setConfidence", &Class::setConfidence, "confidence"_a);
    cls.def("setOutputPolygons", &Class::setOutputPolygons, "output_polygons"_a);
    cls.def("setDegree", &Class::setDegree, "degree"_a);
    cls.def("setManifold", &Class::setManifold, "manifold"_a);
    cls.def("getDepth", &Class::getDepth);
    cls.def("getMinDepth", &Class::getMinDepth);
    cls.def("getPointWeight", &Class::getPointWeight);
    cls.def("getScale", &Class::getScale);
    cls.def("getSolverDivide", &Class::getSolverDivide);
    cls.def("getIsoDivide", &Class::getIsoDivide);
    cls.def("getSamplesPerNode", &Class::getSamplesPerNode);
    cls.def("getConfidence", &Class::getConfidence);
    cls.def("getOutputPolygons", &Class::getOutputPolygons);
    cls.def("getDegree", &Class::getDegree);
    cls.def("getManifold", &Class::getManifold);
        
}

void defineSurfacePoissonFunctions(py::module &m) {
}

void defineSurfacePoissonClasses(py::module &sub_module) {
    py::module sub_module_Poisson = sub_module.def_submodule("Poisson", "Submodule Poisson");
    defineSurfacePoisson<pcl::PointNormal>(sub_module_Poisson, "PointNormal");
    defineSurfacePoisson<pcl::PointXYZINormal>(sub_module_Poisson, "PointXYZINormal");
    defineSurfacePoisson<pcl::PointXYZRGBNormal>(sub_module_Poisson, "PointXYZRGBNormal");
}