
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/surface/processing.h>



void defineSurfaceMeshProcessing(py::module &m) {
    using Class = pcl::MeshProcessing;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PolygonMeshConstPtr = Class::PolygonMeshConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "MeshProcessing");
    cls.def("process", &Class::process, "output"_a);
    cls.def("setInputMesh", &Class::setInputMesh, "input"_a);
    cls.def("getInputMesh", &Class::getInputMesh);
}

template <typename PointInT, typename PointOutT>
void defineSurfaceCloudSurfaceProcessing(py::module &m, std::string const & suffix) {
    using Class = pcl::CloudSurfaceProcessing<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("process", &Class::process, "output"_a);
        
}

void defineSurfaceProcessingFunctions(py::module &m) {
}

void defineSurfaceProcessingClasses(py::module &sub_module) {
    defineSurfaceMeshProcessing(sub_module);
    py::module sub_module_CloudSurfaceProcessing = sub_module.def_submodule("CloudSurfaceProcessing", "Submodule CloudSurfaceProcessing");
    defineSurfaceCloudSurfaceProcessing<pcl::PointNormal, pcl::PointNormal>(sub_module_CloudSurfaceProcessing, "PointNormal_PointNormal");
    defineSurfaceCloudSurfaceProcessing<pcl::PointNormal, pcl::PointXYZ>(sub_module_CloudSurfaceProcessing, "PointNormal_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<pcl::PointNormal, pcl::PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointNormal_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<pcl::PointNormal, pcl::PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointNormal_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointNormal_PointXYZRGBNormal");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZ, pcl::PointNormal>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointNormal");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZ, pcl::PointXYZ>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZ, pcl::PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZ, pcl::PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointXYZRGBNormal");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointNormal");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGB, pcl::PointXYZ>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGB, pcl::PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGB, pcl::PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointXYZRGBNormal");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointNormal");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBA, pcl::PointXYZ>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBA, pcl::PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointXYZRGBNormal");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBNormal, pcl::PointNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointNormal");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBNormal, pcl::PointXYZ>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBNormal, pcl::PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointXYZRGBNormal");
}