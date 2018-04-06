
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/reconstruction.h>



template <typename PointInT>
void defineSurfacePCLSurfaceBase(py::module &m, std::string const & suffix) {
    using Class = PCLSurfaceBase<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using KdTree = Class::KdTree;
    using KdTreePtr = Class::KdTreePtr;
    py::class_<Class, PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def_property("search_method", &Class::getSearchMethod, &Class::setSearchMethod);
    cls.def("reconstruct", py::overload_cast<pcl::PolygonMesh &> (&Class::reconstruct));
        
}

template <typename PointInT>
void defineSurfaceMeshConstruction(py::module &m, std::string const & suffix) {
    using Class = MeshConstruction<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PCLSurfaceBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("reconstruct", py::overload_cast<pcl::PolygonMesh &> (&Class::reconstruct));
    cls.def("reconstruct", py::overload_cast<std::vector<pcl::Vertices> &> (&Class::reconstruct));
        
}

template <typename PointInT>
void defineSurfaceSurfaceReconstruction(py::module &m, std::string const & suffix) {
    using Class = SurfaceReconstruction<PointInT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PCLSurfaceBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("reconstruct", py::overload_cast<pcl::PolygonMesh &> (&Class::reconstruct));
    cls.def("reconstruct", py::overload_cast<pcl::PointCloud<PointInT> &, std::vector<pcl::Vertices> &> (&Class::reconstruct));
        
}

void defineSurfaceReconstructionClasses(py::module &sub_module) {
    py::module sub_module_PCLSurfaceBase = sub_module.def_submodule("PCLSurfaceBase", "Submodule PCLSurfaceBase");
    defineSurfacePCLSurfaceBase<InterestPoint>(sub_module_PCLSurfaceBase, "InterestPoint");
    defineSurfacePCLSurfaceBase<PointDEM>(sub_module_PCLSurfaceBase, "PointDEM");
    defineSurfacePCLSurfaceBase<PointNormal>(sub_module_PCLSurfaceBase, "PointNormal");
    defineSurfacePCLSurfaceBase<PointSurfel>(sub_module_PCLSurfaceBase, "PointSurfel");
    defineSurfacePCLSurfaceBase<PointWithRange>(sub_module_PCLSurfaceBase, "PointWithRange");
    defineSurfacePCLSurfaceBase<PointWithScale>(sub_module_PCLSurfaceBase, "PointWithScale");
    defineSurfacePCLSurfaceBase<PointWithViewpoint>(sub_module_PCLSurfaceBase, "PointWithViewpoint");
    defineSurfacePCLSurfaceBase<PointXYZ>(sub_module_PCLSurfaceBase, "PointXYZ");
    defineSurfacePCLSurfaceBase<PointXYZHSV>(sub_module_PCLSurfaceBase, "PointXYZHSV");
    defineSurfacePCLSurfaceBase<PointXYZI>(sub_module_PCLSurfaceBase, "PointXYZI");
    defineSurfacePCLSurfaceBase<PointXYZINormal>(sub_module_PCLSurfaceBase, "PointXYZINormal");
    defineSurfacePCLSurfaceBase<PointXYZL>(sub_module_PCLSurfaceBase, "PointXYZL");
    defineSurfacePCLSurfaceBase<PointXYZLNormal>(sub_module_PCLSurfaceBase, "PointXYZLNormal");
    defineSurfacePCLSurfaceBase<PointXYZRGB>(sub_module_PCLSurfaceBase, "PointXYZRGB");
    defineSurfacePCLSurfaceBase<PointXYZRGBA>(sub_module_PCLSurfaceBase, "PointXYZRGBA");
    defineSurfacePCLSurfaceBase<PointXYZRGBL>(sub_module_PCLSurfaceBase, "PointXYZRGBL");
    defineSurfacePCLSurfaceBase<PointXYZRGBNormal>(sub_module_PCLSurfaceBase, "PointXYZRGBNormal");
    py::module sub_module_MeshConstruction = sub_module.def_submodule("MeshConstruction", "Submodule MeshConstruction");
    defineSurfaceMeshConstruction<InterestPoint>(sub_module_MeshConstruction, "InterestPoint");
    defineSurfaceMeshConstruction<PointDEM>(sub_module_MeshConstruction, "PointDEM");
    defineSurfaceMeshConstruction<PointNormal>(sub_module_MeshConstruction, "PointNormal");
    defineSurfaceMeshConstruction<PointSurfel>(sub_module_MeshConstruction, "PointSurfel");
    defineSurfaceMeshConstruction<PointWithRange>(sub_module_MeshConstruction, "PointWithRange");
    defineSurfaceMeshConstruction<PointWithScale>(sub_module_MeshConstruction, "PointWithScale");
    defineSurfaceMeshConstruction<PointWithViewpoint>(sub_module_MeshConstruction, "PointWithViewpoint");
    defineSurfaceMeshConstruction<PointXYZ>(sub_module_MeshConstruction, "PointXYZ");
    defineSurfaceMeshConstruction<PointXYZHSV>(sub_module_MeshConstruction, "PointXYZHSV");
    defineSurfaceMeshConstruction<PointXYZI>(sub_module_MeshConstruction, "PointXYZI");
    defineSurfaceMeshConstruction<PointXYZINormal>(sub_module_MeshConstruction, "PointXYZINormal");
    defineSurfaceMeshConstruction<PointXYZL>(sub_module_MeshConstruction, "PointXYZL");
    defineSurfaceMeshConstruction<PointXYZLNormal>(sub_module_MeshConstruction, "PointXYZLNormal");
    defineSurfaceMeshConstruction<PointXYZRGB>(sub_module_MeshConstruction, "PointXYZRGB");
    defineSurfaceMeshConstruction<PointXYZRGBA>(sub_module_MeshConstruction, "PointXYZRGBA");
    defineSurfaceMeshConstruction<PointXYZRGBL>(sub_module_MeshConstruction, "PointXYZRGBL");
    defineSurfaceMeshConstruction<PointXYZRGBNormal>(sub_module_MeshConstruction, "PointXYZRGBNormal");
    py::module sub_module_SurfaceReconstruction = sub_module.def_submodule("SurfaceReconstruction", "Submodule SurfaceReconstruction");
    defineSurfaceSurfaceReconstruction<PointNormal>(sub_module_SurfaceReconstruction, "PointNormal");
    defineSurfaceSurfaceReconstruction<PointXYZINormal>(sub_module_SurfaceReconstruction, "PointXYZINormal");
    defineSurfaceSurfaceReconstruction<PointXYZRGBNormal>(sub_module_SurfaceReconstruction, "PointXYZRGBNormal");
}