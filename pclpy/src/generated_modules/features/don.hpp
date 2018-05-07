
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/features/don.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesDifferenceOfNormalsEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::DifferenceOfNormalsEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::Feature<PointInT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computeFeature", &Class::computeFeature, "output"_a);
    cls.def("initCompute", &Class::initCompute);
    cls.def("setNormalScaleSmall", &Class::setNormalScaleSmall, "normals"_a);
    cls.def("setNormalScaleLarge", &Class::setNormalScaleLarge, "normals"_a);
        
}

void defineFeaturesDonFunctions(py::module &m) {
}

void defineFeaturesDonClasses(py::module &sub_module) {
    py::module sub_module_DifferenceOfNormalsEstimation = sub_module.def_submodule("DifferenceOfNormalsEstimation", "Submodule DifferenceOfNormalsEstimation");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointNormal, pcl::Normal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointNormal_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointNormal, pcl::Normal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointNormal_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointNormal, pcl::PointNormal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointNormal_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointNormal_PointNormal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::Normal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZ_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::Normal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZ_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZ_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZ_PointNormal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZI, pcl::Normal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZI_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZI, pcl::Normal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZI_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZI_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZI_PointNormal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGB_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGB_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGB_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGB_PointNormal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGBA_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGBA_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGBA_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGBA_PointNormal_PointNormal");
}