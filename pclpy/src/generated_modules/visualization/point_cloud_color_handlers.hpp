
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/visualization/point_cloud_color_handlers.h>

using namespace pcl::visualization;


template <typename PointT>
void defineVisualizationPointCloudColorHandler(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudColorHandler<PointT>;
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
    cls.def("getColor", &Class::getColor, "scalars"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudColorHandlerCustom(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudColorHandlerCustom<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudColorHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<double, double, double>(), "r"_a, "g"_a, "b"_a);
    cls.def(py::init<PointCloudColorHandler<PointT>::PointCloud::ConstPtr, double, double, double>(), "cloud"_a, "r"_a, "g"_a, "b"_a);
    cls.def("getName", &Class::getName);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getColor", &Class::getColor, "scalars"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudColorHandlerGenericField(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudColorHandlerGenericField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudColorHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<std::string>(), "field_name"_a);
    cls.def(py::init<PointCloudColorHandler<PointT>::PointCloud::ConstPtr, std::string>(), "cloud"_a, "field_name"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getColor", &Class::getColor, "scalars"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudColorHandlerHSVField(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudColorHandlerHSVField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudColorHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<PointCloudColorHandler<PointT>::PointCloud::ConstPtr>(), "cloud"_a);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getColor", &Class::getColor, "scalars"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudColorHandlerLabelField(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudColorHandlerLabelField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudColorHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "static_mapping"_a=true);
    cls.def(py::init<PointCloudColorHandler<PointT>::PointCloud::ConstPtr, bool>(), "cloud"_a, "static_mapping"_a=true);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getColor", &Class::getColor, "scalars"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudColorHandlerRGBAField(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudColorHandlerRGBAField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudColorHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<PointCloudColorHandler<PointT>::PointCloud::ConstPtr>(), "cloud"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getColor", &Class::getColor, "scalars"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudColorHandlerRGBField(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudColorHandlerRGBField<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudColorHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<PointCloudColorHandler<PointT>::PointCloud::ConstPtr>(), "cloud"_a);
    cls.def("setInputCloud", &Class::setInputCloud, "cloud"_a);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getColor", &Class::getColor, "scalars"_a);
        
}

template <typename PointT>
void defineVisualizationPointCloudColorHandlerRandom(py::module &m, std::string const & suffix) {
    using Class = pcl::visualization::PointCloudColorHandlerRandom<PointT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::visualization::PointCloudColorHandler<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def(py::init<PointCloudColorHandler<PointT>::PointCloud::ConstPtr>(), "cloud"_a);
    cls.def("getName", &Class::getName);
    cls.def("getFieldName", &Class::getFieldName);
    cls.def("getColor", &Class::getColor, "scalars"_a);
        
}

void defineVisualizationPointCloudColorHandlersFunctions(py::module &m) {
}

void defineVisualizationPointCloudColorHandlersClasses(py::module &sub_module) {
    py::module sub_module_PointCloudColorHandler = sub_module.def_submodule("PointCloudColorHandler", "Submodule PointCloudColorHandler");
    defineVisualizationPointCloudColorHandler<pcl::PointNormal>(sub_module_PointCloudColorHandler, "PointNormal");
    defineVisualizationPointCloudColorHandler<pcl::PointSurfel>(sub_module_PointCloudColorHandler, "PointSurfel");
    defineVisualizationPointCloudColorHandler<pcl::PointWithRange>(sub_module_PointCloudColorHandler, "PointWithRange");
    defineVisualizationPointCloudColorHandler<pcl::PointXYZ>(sub_module_PointCloudColorHandler, "PointXYZ");
    defineVisualizationPointCloudColorHandler<pcl::PointXYZHSV>(sub_module_PointCloudColorHandler, "PointXYZHSV");
    defineVisualizationPointCloudColorHandler<pcl::PointXYZI>(sub_module_PointCloudColorHandler, "PointXYZI");
    defineVisualizationPointCloudColorHandler<pcl::PointXYZL>(sub_module_PointCloudColorHandler, "PointXYZL");
    defineVisualizationPointCloudColorHandler<pcl::PointXYZLNormal>(sub_module_PointCloudColorHandler, "PointXYZLNormal");
    defineVisualizationPointCloudColorHandler<pcl::PointXYZRGB>(sub_module_PointCloudColorHandler, "PointXYZRGB");
    defineVisualizationPointCloudColorHandler<pcl::PointXYZRGBA>(sub_module_PointCloudColorHandler, "PointXYZRGBA");
    defineVisualizationPointCloudColorHandler<pcl::PointXYZRGBL>(sub_module_PointCloudColorHandler, "PointXYZRGBL");
    defineVisualizationPointCloudColorHandler<pcl::PointXYZRGBNormal>(sub_module_PointCloudColorHandler, "PointXYZRGBNormal");
    py::module sub_module_PointCloudColorHandlerCustom = sub_module.def_submodule("PointCloudColorHandlerCustom", "Submodule PointCloudColorHandlerCustom");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointNormal>(sub_module_PointCloudColorHandlerCustom, "PointNormal");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointSurfel>(sub_module_PointCloudColorHandlerCustom, "PointSurfel");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointWithRange>(sub_module_PointCloudColorHandlerCustom, "PointWithRange");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointXYZ>(sub_module_PointCloudColorHandlerCustom, "PointXYZ");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointXYZI>(sub_module_PointCloudColorHandlerCustom, "PointXYZI");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointXYZL>(sub_module_PointCloudColorHandlerCustom, "PointXYZL");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointXYZRGB>(sub_module_PointCloudColorHandlerCustom, "PointXYZRGB");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointXYZRGBA>(sub_module_PointCloudColorHandlerCustom, "PointXYZRGBA");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointXYZRGBL>(sub_module_PointCloudColorHandlerCustom, "PointXYZRGBL");
    defineVisualizationPointCloudColorHandlerCustom<pcl::PointXYZRGBNormal>(sub_module_PointCloudColorHandlerCustom, "PointXYZRGBNormal");
    py::module sub_module_PointCloudColorHandlerGenericField = sub_module.def_submodule("PointCloudColorHandlerGenericField", "Submodule PointCloudColorHandlerGenericField");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointNormal>(sub_module_PointCloudColorHandlerGenericField, "PointNormal");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointSurfel>(sub_module_PointCloudColorHandlerGenericField, "PointSurfel");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointWithRange>(sub_module_PointCloudColorHandlerGenericField, "PointWithRange");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointXYZ>(sub_module_PointCloudColorHandlerGenericField, "PointXYZ");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointXYZI>(sub_module_PointCloudColorHandlerGenericField, "PointXYZI");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointXYZL>(sub_module_PointCloudColorHandlerGenericField, "PointXYZL");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointXYZRGB>(sub_module_PointCloudColorHandlerGenericField, "PointXYZRGB");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointXYZRGBA>(sub_module_PointCloudColorHandlerGenericField, "PointXYZRGBA");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointXYZRGBL>(sub_module_PointCloudColorHandlerGenericField, "PointXYZRGBL");
    defineVisualizationPointCloudColorHandlerGenericField<pcl::PointXYZRGBNormal>(sub_module_PointCloudColorHandlerGenericField, "PointXYZRGBNormal");
    py::module sub_module_PointCloudColorHandlerHSVField = sub_module.def_submodule("PointCloudColorHandlerHSVField", "Submodule PointCloudColorHandlerHSVField");
    defineVisualizationPointCloudColorHandlerHSVField<pcl::PointXYZHSV>(sub_module_PointCloudColorHandlerHSVField, "PointXYZHSV");
    py::module sub_module_PointCloudColorHandlerLabelField = sub_module.def_submodule("PointCloudColorHandlerLabelField", "Submodule PointCloudColorHandlerLabelField");
    defineVisualizationPointCloudColorHandlerLabelField<pcl::PointXYZL>(sub_module_PointCloudColorHandlerLabelField, "PointXYZL");
    defineVisualizationPointCloudColorHandlerLabelField<pcl::PointXYZLNormal>(sub_module_PointCloudColorHandlerLabelField, "PointXYZLNormal");
    defineVisualizationPointCloudColorHandlerLabelField<pcl::PointXYZRGBL>(sub_module_PointCloudColorHandlerLabelField, "PointXYZRGBL");
    py::module sub_module_PointCloudColorHandlerRGBAField = sub_module.def_submodule("PointCloudColorHandlerRGBAField", "Submodule PointCloudColorHandlerRGBAField");
    defineVisualizationPointCloudColorHandlerRGBAField<pcl::PointXYZRGBA>(sub_module_PointCloudColorHandlerRGBAField, "PointXYZRGBA");
    py::module sub_module_PointCloudColorHandlerRGBField = sub_module.def_submodule("PointCloudColorHandlerRGBField", "Submodule PointCloudColorHandlerRGBField");
    defineVisualizationPointCloudColorHandlerRGBField<pcl::PointXYZRGB>(sub_module_PointCloudColorHandlerRGBField, "PointXYZRGB");
    defineVisualizationPointCloudColorHandlerRGBField<pcl::PointXYZRGBL>(sub_module_PointCloudColorHandlerRGBField, "PointXYZRGBL");
    defineVisualizationPointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(sub_module_PointCloudColorHandlerRGBField, "PointXYZRGBNormal");
    py::module sub_module_PointCloudColorHandlerRandom = sub_module.def_submodule("PointCloudColorHandlerRandom", "Submodule PointCloudColorHandlerRandom");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointNormal>(sub_module_PointCloudColorHandlerRandom, "PointNormal");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointSurfel>(sub_module_PointCloudColorHandlerRandom, "PointSurfel");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointWithRange>(sub_module_PointCloudColorHandlerRandom, "PointWithRange");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointXYZ>(sub_module_PointCloudColorHandlerRandom, "PointXYZ");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointXYZI>(sub_module_PointCloudColorHandlerRandom, "PointXYZI");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointXYZL>(sub_module_PointCloudColorHandlerRandom, "PointXYZL");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointXYZRGB>(sub_module_PointCloudColorHandlerRandom, "PointXYZRGB");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointXYZRGBA>(sub_module_PointCloudColorHandlerRandom, "PointXYZRGBA");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointXYZRGBL>(sub_module_PointCloudColorHandlerRandom, "PointXYZRGBL");
    defineVisualizationPointCloudColorHandlerRandom<pcl::PointXYZRGBNormal>(sub_module_PointCloudColorHandlerRandom, "PointXYZRGBNormal");
}