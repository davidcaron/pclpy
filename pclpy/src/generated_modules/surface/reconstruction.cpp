
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

#include <pcl/surface/reconstruction.h>



template <typename PointInT>
void defineSurfacePCLSurfaceBase(py::module &m, std::string const & suffix) {
    using Class = pcl::PCLSurfaceBase<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    py::class_<Class, pcl::PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("reconstruct", py::overload_cast<pcl::PolygonMesh &> (&Class::reconstruct), "output"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "tree"_a);
    cls.def("getSearchMethod", &Class::getSearchMethod);
        
}

template <typename PointInT>
void defineSurfaceMeshConstruction(py::module &m, std::string const & suffix) {
    using Class = pcl::MeshConstruction<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::PCLSurfaceBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("reconstruct", py::overload_cast<pcl::PolygonMesh &> (&Class::reconstruct), "output"_a);
    cls.def("reconstruct", py::overload_cast<std::vector<pcl::Vertices> &> (&Class::reconstruct), "polygons"_a);
        
}

template <typename PointInT>
void defineSurfaceSurfaceReconstruction(py::module &m, std::string const & suffix) {
    using Class = pcl::SurfaceReconstruction<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::PCLSurfaceBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("reconstruct", py::overload_cast<pcl::PolygonMesh &> (&Class::reconstruct), "output"_a);
    cls.def("reconstruct", py::overload_cast<pcl::PointCloud<PointInT> &, std::vector<pcl::Vertices> &> (&Class::reconstruct), "points"_a, "polygons"_a);
        
}

void defineSurfaceReconstructionFunctions(py::module &m) {
}

void defineSurfaceReconstructionClasses(py::module &sub_module) {
    py::module sub_module_PCLSurfaceBase = sub_module.def_submodule("PCLSurfaceBase", "Submodule PCLSurfaceBase");
    defineSurfacePCLSurfaceBase<pcl::InterestPoint>(sub_module_PCLSurfaceBase, "InterestPoint");
    defineSurfacePCLSurfaceBase<pcl::PointDEM>(sub_module_PCLSurfaceBase, "PointDEM");
    defineSurfacePCLSurfaceBase<pcl::PointNormal>(sub_module_PCLSurfaceBase, "PointNormal");
    defineSurfacePCLSurfaceBase<pcl::PointSurfel>(sub_module_PCLSurfaceBase, "PointSurfel");
    defineSurfacePCLSurfaceBase<pcl::PointWithRange>(sub_module_PCLSurfaceBase, "PointWithRange");
    defineSurfacePCLSurfaceBase<pcl::PointWithScale>(sub_module_PCLSurfaceBase, "PointWithScale");
    defineSurfacePCLSurfaceBase<pcl::PointWithViewpoint>(sub_module_PCLSurfaceBase, "PointWithViewpoint");
    defineSurfacePCLSurfaceBase<pcl::PointXYZ>(sub_module_PCLSurfaceBase, "PointXYZ");
    defineSurfacePCLSurfaceBase<pcl::PointXYZHSV>(sub_module_PCLSurfaceBase, "PointXYZHSV");
    defineSurfacePCLSurfaceBase<pcl::PointXYZI>(sub_module_PCLSurfaceBase, "PointXYZI");
    defineSurfacePCLSurfaceBase<pcl::PointXYZINormal>(sub_module_PCLSurfaceBase, "PointXYZINormal");
    defineSurfacePCLSurfaceBase<pcl::PointXYZL>(sub_module_PCLSurfaceBase, "PointXYZL");
    defineSurfacePCLSurfaceBase<pcl::PointXYZLNormal>(sub_module_PCLSurfaceBase, "PointXYZLNormal");
    defineSurfacePCLSurfaceBase<pcl::PointXYZRGB>(sub_module_PCLSurfaceBase, "PointXYZRGB");
    defineSurfacePCLSurfaceBase<pcl::PointXYZRGBA>(sub_module_PCLSurfaceBase, "PointXYZRGBA");
    defineSurfacePCLSurfaceBase<pcl::PointXYZRGBL>(sub_module_PCLSurfaceBase, "PointXYZRGBL");
    defineSurfacePCLSurfaceBase<pcl::PointXYZRGBNormal>(sub_module_PCLSurfaceBase, "PointXYZRGBNormal");
    py::module sub_module_MeshConstruction = sub_module.def_submodule("MeshConstruction", "Submodule MeshConstruction");
    defineSurfaceMeshConstruction<pcl::InterestPoint>(sub_module_MeshConstruction, "InterestPoint");
    defineSurfaceMeshConstruction<pcl::PointDEM>(sub_module_MeshConstruction, "PointDEM");
    defineSurfaceMeshConstruction<pcl::PointNormal>(sub_module_MeshConstruction, "PointNormal");
    defineSurfaceMeshConstruction<pcl::PointSurfel>(sub_module_MeshConstruction, "PointSurfel");
    defineSurfaceMeshConstruction<pcl::PointWithRange>(sub_module_MeshConstruction, "PointWithRange");
    defineSurfaceMeshConstruction<pcl::PointWithScale>(sub_module_MeshConstruction, "PointWithScale");
    defineSurfaceMeshConstruction<pcl::PointWithViewpoint>(sub_module_MeshConstruction, "PointWithViewpoint");
    defineSurfaceMeshConstruction<pcl::PointXYZ>(sub_module_MeshConstruction, "PointXYZ");
    defineSurfaceMeshConstruction<pcl::PointXYZHSV>(sub_module_MeshConstruction, "PointXYZHSV");
    defineSurfaceMeshConstruction<pcl::PointXYZI>(sub_module_MeshConstruction, "PointXYZI");
    defineSurfaceMeshConstruction<pcl::PointXYZINormal>(sub_module_MeshConstruction, "PointXYZINormal");
    defineSurfaceMeshConstruction<pcl::PointXYZL>(sub_module_MeshConstruction, "PointXYZL");
    defineSurfaceMeshConstruction<pcl::PointXYZLNormal>(sub_module_MeshConstruction, "PointXYZLNormal");
    defineSurfaceMeshConstruction<pcl::PointXYZRGB>(sub_module_MeshConstruction, "PointXYZRGB");
    defineSurfaceMeshConstruction<pcl::PointXYZRGBA>(sub_module_MeshConstruction, "PointXYZRGBA");
    defineSurfaceMeshConstruction<pcl::PointXYZRGBL>(sub_module_MeshConstruction, "PointXYZRGBL");
    defineSurfaceMeshConstruction<pcl::PointXYZRGBNormal>(sub_module_MeshConstruction, "PointXYZRGBNormal");
    py::module sub_module_SurfaceReconstruction = sub_module.def_submodule("SurfaceReconstruction", "Submodule SurfaceReconstruction");
    defineSurfaceSurfaceReconstruction<pcl::PointNormal>(sub_module_SurfaceReconstruction, "PointNormal");
    defineSurfaceSurfaceReconstruction<pcl::PointXYZINormal>(sub_module_SurfaceReconstruction, "PointXYZINormal");
    defineSurfaceSurfaceReconstruction<pcl::PointXYZRGBNormal>(sub_module_SurfaceReconstruction, "PointXYZRGBNormal");
}