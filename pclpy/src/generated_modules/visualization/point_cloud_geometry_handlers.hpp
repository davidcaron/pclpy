
#include <pcl/visualization/point_cloud_geometry_handlers.h>

using namespace pcl::visualization;


template <typename PointT>
void defineVisualizationPointCloudGeometryHandler(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudGeometryHandler<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def("isCapable", &Class::isCapable);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getName", &Class::getName);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getGeometry", &Class::getGeometry, "points"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudGeometryHandlerCustom(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudGeometryHandlerCustom<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudGeometryHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloudConstPtr, std::string, std::string, std::string>(), "cloud"_a, "x_field_name"_a, "y_field_name"_a, "z_field_name"_a);
    cls.def("getName", &Class::getName);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getGeometry", &Class::getGeometry, "points"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudGeometryHandlerSurfaceNormal(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudGeometryHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloudConstPtr>(), "cloud"_a);
    cls.def("getName", &Class::getName);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getGeometry", &Class::getGeometry, "points"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudGeometryHandlerXYZ(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudGeometryHandlerXYZ<PointT>;
    using PointCloud = Class::PointCloud;
    using PointCloudPtr = Class::PointCloudPtr;
    using PointCloudConstPtr = Class::PointCloudConstPtr;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudGeometryHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloudConstPtr>(), "cloud"_a);
    cls.def("getName", &Class::getName);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getGeometry", &Class::getGeometry, "points"_a);
        
}

void defineVisualizationPointCloudGeometryHandlersFunctions(py::module &m) {
}

void defineVisualizationPointCloudGeometryHandlersClasses(py::module &sub_module) {
    py::module sub_module_PointCloudGeometryHandler = sub_module.def_submodule("PointCloudGeometryHandler", "Submodule PointCloudGeometryHandler");
    defineVisualizationPointCloudGeometryHandler<pcl::Normal>(sub_module_PointCloudGeometryHandler, "Normal");
    defineVisualizationPointCloudGeometryHandler<pcl::PointNormal>(sub_module_PointCloudGeometryHandler, "PointNormal");
    defineVisualizationPointCloudGeometryHandler<pcl::PointSurfel>(sub_module_PointCloudGeometryHandler, "PointSurfel");
    defineVisualizationPointCloudGeometryHandler<pcl::PointWithRange>(sub_module_PointCloudGeometryHandler, "PointWithRange");
    defineVisualizationPointCloudGeometryHandler<pcl::PointXYZ>(sub_module_PointCloudGeometryHandler, "PointXYZ");
    defineVisualizationPointCloudGeometryHandler<pcl::PointXYZI>(sub_module_PointCloudGeometryHandler, "PointXYZI");
    defineVisualizationPointCloudGeometryHandler<pcl::PointXYZL>(sub_module_PointCloudGeometryHandler, "PointXYZL");
    defineVisualizationPointCloudGeometryHandler<pcl::PointXYZRGB>(sub_module_PointCloudGeometryHandler, "PointXYZRGB");
    defineVisualizationPointCloudGeometryHandler<pcl::PointXYZRGBA>(sub_module_PointCloudGeometryHandler, "PointXYZRGBA");
    defineVisualizationPointCloudGeometryHandler<pcl::PointXYZRGBL>(sub_module_PointCloudGeometryHandler, "PointXYZRGBL");
    defineVisualizationPointCloudGeometryHandler<pcl::PointXYZRGBNormal>(sub_module_PointCloudGeometryHandler, "PointXYZRGBNormal");
    py::module sub_module_PointCloudGeometryHandlerSurfaceNormal = sub_module.def_submodule("PointCloudGeometryHandlerSurfaceNormal", "Submodule PointCloudGeometryHandlerSurfaceNormal");
    defineVisualizationPointCloudGeometryHandlerSurfaceNormal<pcl::Normal>(sub_module_PointCloudGeometryHandlerSurfaceNormal, "Normal");
    defineVisualizationPointCloudGeometryHandlerSurfaceNormal<pcl::PointNormal>(sub_module_PointCloudGeometryHandlerSurfaceNormal, "PointNormal");
    defineVisualizationPointCloudGeometryHandlerSurfaceNormal<pcl::PointXYZRGBNormal>(sub_module_PointCloudGeometryHandlerSurfaceNormal, "PointXYZRGBNormal");
    py::module sub_module_PointCloudGeometryHandlerXYZ = sub_module.def_submodule("PointCloudGeometryHandlerXYZ", "Submodule PointCloudGeometryHandlerXYZ");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointNormal>(sub_module_PointCloudGeometryHandlerXYZ, "PointNormal");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointSurfel>(sub_module_PointCloudGeometryHandlerXYZ, "PointSurfel");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointWithRange>(sub_module_PointCloudGeometryHandlerXYZ, "PointWithRange");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointXYZ>(sub_module_PointCloudGeometryHandlerXYZ, "PointXYZ");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointXYZI>(sub_module_PointCloudGeometryHandlerXYZ, "PointXYZI");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointXYZL>(sub_module_PointCloudGeometryHandlerXYZ, "PointXYZL");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointXYZRGB>(sub_module_PointCloudGeometryHandlerXYZ, "PointXYZRGB");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointXYZRGBA>(sub_module_PointCloudGeometryHandlerXYZ, "PointXYZRGBA");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointXYZRGBL>(sub_module_PointCloudGeometryHandlerXYZ, "PointXYZRGBL");
    defineVisualizationPointCloudGeometryHandlerXYZ<pcl::PointXYZRGBNormal>(sub_module_PointCloudGeometryHandlerXYZ, "PointXYZRGBNormal");
}