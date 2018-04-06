
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/processing.h>



void defineSurfaceMeshProcessing(py::module &m) {
    using Class = MeshProcessing;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PolygonMeshConstPtr = Class::PolygonMeshConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, "MeshProcessing");
    cls.def_property("input_mesh", &Class::getInputMesh, &Class::setInputMesh);
    cls.def("process", &Class::process);
}

template <typename PointInT, typename PointOutT>
void defineSurfaceCloudSurfaceProcessing(py::module &m, std::string const & suffix) {
    using Class = CloudSurfaceProcessing<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, PCLBase<PointInT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("process", &Class::process);
        
}

void defineSurfaceProcessingClasses(py::module &sub_module) {
    defineSurfaceMeshProcessing(sub_module);
    py::module sub_module_CloudSurfaceProcessing = sub_module.def_submodule("CloudSurfaceProcessing", "Submodule CloudSurfaceProcessing");
    defineSurfaceCloudSurfaceProcessing<PointNormal, PointNormal>(sub_module_CloudSurfaceProcessing, "PointNormal_PointNormal");
    defineSurfaceCloudSurfaceProcessing<PointNormal, PointXYZ>(sub_module_CloudSurfaceProcessing, "PointNormal_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<PointNormal, PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointNormal_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<PointNormal, PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointNormal_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<PointNormal, PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointNormal_PointXYZRGBNormal");
    defineSurfaceCloudSurfaceProcessing<PointXYZ, PointNormal>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointNormal");
    defineSurfaceCloudSurfaceProcessing<PointXYZ, PointXYZ>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<PointXYZ, PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<PointXYZ, PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<PointXYZ, PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointXYZ_PointXYZRGBNormal");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGB, PointNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointNormal");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGB, PointXYZ>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGB, PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGB, PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGB, PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGB_PointXYZRGBNormal");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBA, PointNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointNormal");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBA, PointXYZ>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBA, PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBA, PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBA, PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGBA_PointXYZRGBNormal");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBNormal, PointNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointNormal");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBNormal, PointXYZ>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointXYZ");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBNormal, PointXYZRGB>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointXYZRGB");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBNormal, PointXYZRGBA>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointXYZRGBA");
    defineSurfaceCloudSurfaceProcessing<PointXYZRGBNormal, PointXYZRGBNormal>(sub_module_CloudSurfaceProcessing, "PointXYZRGBNormal_PointXYZRGBNormal");
}