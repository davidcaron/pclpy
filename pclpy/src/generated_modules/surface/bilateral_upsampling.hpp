
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/bilateral_upsampling.h>



template <typename PointInT, typename PointOutT>
void defineSurfaceBilateralUpsampling(py::module &m, std::string const & suffix) {
    using Class = BilateralUpsampling<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, CloudSurfaceProcessing<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_property("window_size", &Class::getWindowSize, &Class::setWindowSize);
    cls.def_property("sigma_color", &Class::getSigmaColor, &Class::setSigmaColor);
    cls.def_property("sigma_depth", &Class::getSigmaDepth, &Class::setSigmaDepth);
    cls.def_property("projection_matrix", &Class::getProjectionMatrix, &Class::setProjectionMatrix);
    cls.def_readonly("kinect_vga_projection_matrix", &Class::KinectVGAProjectionMatrix);
    cls.def_readonly("kinect_sxga_projection_matrix", &Class::KinectSXGAProjectionMatrix);
    cls.def("process", &Class::process);
        
}

void defineSurfaceBilateralUpsamplingClasses(py::module &sub_module) {
    py::module sub_module_BilateralUpsampling = sub_module.def_submodule("BilateralUpsampling", "Submodule BilateralUpsampling");
    defineSurfaceBilateralUpsampling<PointXYZRGB, PointXYZRGB>(sub_module_BilateralUpsampling, "PointXYZRGB_PointXYZRGB");
    defineSurfaceBilateralUpsampling<PointXYZRGB, PointXYZRGBA>(sub_module_BilateralUpsampling, "PointXYZRGB_PointXYZRGBA");
    defineSurfaceBilateralUpsampling<PointXYZRGBA, PointXYZRGB>(sub_module_BilateralUpsampling, "PointXYZRGBA_PointXYZRGB");
    defineSurfaceBilateralUpsampling<PointXYZRGBA, PointXYZRGBA>(sub_module_BilateralUpsampling, "PointXYZRGBA_PointXYZRGBA");
}