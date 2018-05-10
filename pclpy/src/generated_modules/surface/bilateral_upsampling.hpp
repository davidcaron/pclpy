
#include <pcl/surface/bilateral_upsampling.h>



template <typename PointInT, typename PointOutT>
void defineSurfaceBilateralUpsampling(py::module &m, std::string const & suffix) {
    using Class = pcl::BilateralUpsampling<PointInT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::CloudSurfaceProcessing<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def_readwrite("KinectVGAProjectionMatrix", &Class::KinectVGAProjectionMatrix);
    cls.def_readwrite("KinectSXGAProjectionMatrix", &Class::KinectSXGAProjectionMatrix);
    cls.def("process", &Class::process, "output"_a);
    cls.def("setWindowSize", &Class::setWindowSize, "window_size"_a);
    cls.def("setSigmaColor", &Class::setSigmaColor, "sigma_color"_a);
    cls.def("setSigmaDepth", &Class::setSigmaDepth, "sigma_depth"_a);
    cls.def("setProjectionMatrix", &Class::setProjectionMatrix, "projection_matrix"_a);
    cls.def("getWindowSize", &Class::getWindowSize);
    cls.def("getSigmaColor", &Class::getSigmaColor);
    cls.def("getSigmaDepth", &Class::getSigmaDepth);
    cls.def("getProjectionMatrix", &Class::getProjectionMatrix);
        
}

void defineSurfaceBilateralUpsamplingFunctions(py::module &m) {
}

void defineSurfaceBilateralUpsamplingClasses(py::module &sub_module) {
    py::module sub_module_BilateralUpsampling = sub_module.def_submodule("BilateralUpsampling", "Submodule BilateralUpsampling");
    defineSurfaceBilateralUpsampling<pcl::PointXYZRGB, pcl::PointXYZRGB>(sub_module_BilateralUpsampling, "PointXYZRGB_PointXYZRGB");
    defineSurfaceBilateralUpsampling<pcl::PointXYZRGB, pcl::PointXYZRGBA>(sub_module_BilateralUpsampling, "PointXYZRGB_PointXYZRGBA");
    defineSurfaceBilateralUpsampling<pcl::PointXYZRGBA, pcl::PointXYZRGB>(sub_module_BilateralUpsampling, "PointXYZRGBA_PointXYZRGB");
    defineSurfaceBilateralUpsampling<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(sub_module_BilateralUpsampling, "PointXYZRGBA_PointXYZRGBA");
}