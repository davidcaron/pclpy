
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/poisson.h>



template<typename PointNT>
void defineSurfacePoisson(py::module &m, std::string const & suffix) {
    using Class = Poisson<PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudPtr = Class::PointCloudPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    py::class_<Class, SurfaceReconstruction<PointNT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("depth", &Class::getDepth, &Class::setDepth);
    cls.def_property("min_depth", &Class::getMinDepth, &Class::setMinDepth);
    cls.def_property("point_weight", &Class::getPointWeight, &Class::setPointWeight);
    cls.def_property("scale", &Class::getScale, &Class::setScale);
    cls.def_property("solver_divide", &Class::getSolverDivide, &Class::setSolverDivide);
    cls.def_property("iso_divide", &Class::getIsoDivide, &Class::setIsoDivide);
    cls.def_property("samples_per_node", &Class::getSamplesPerNode, &Class::setSamplesPerNode);
    cls.def_property("confidence", &Class::getConfidence, &Class::setConfidence);
    cls.def_property("output_polygons", &Class::getOutputPolygons, &Class::setOutputPolygons);
    cls.def_property("degree", &Class::getDegree, &Class::setDegree);
    cls.def_property("manifold", &Class::getManifold, &Class::setManifold);
    cls.def("perform_reconstruction", py::overload_cast<pcl::PolygonMesh &> (&Class::performReconstruction));
    cls.def("perform_reconstruction", py::overload_cast<pcl::PointCloud<PointNT> &, std::vector<pcl::Vertices> &> (&Class::performReconstruction));
        
}

void defineSurfacePoissonClasses(py::module &sub_module) {
    py::module sub_module_Poisson = sub_module.def_submodule("Poisson", "Submodule Poisson");
    defineSurfacePoisson<PointNormal>(sub_module_Poisson, "PointNormal");
    defineSurfacePoisson<PointXYZINormal>(sub_module_Poisson, "PointXYZINormal");
    defineSurfacePoisson<PointXYZRGBNormal>(sub_module_Poisson, "PointXYZRGBNormal");
}