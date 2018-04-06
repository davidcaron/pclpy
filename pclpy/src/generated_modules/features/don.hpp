
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/don.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesDifferenceOfNormalsEstimation(py::module &m, std::string const & suffix) {
    using Class = DifferenceOfNormalsEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, Feature<PointInT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("set_normal_scale_small", &Class::setNormalScaleSmall);
    cls.def("set_normal_scale_large", &Class::setNormalScaleLarge);
    cls.def("compute_feature", &Class::computeFeature);
    cls.def("init_compute", &Class::initCompute);
        
}

void defineFeaturesDonClasses(py::module &sub_module) {
    py::module sub_module_DifferenceOfNormalsEstimation = sub_module.def_submodule("DifferenceOfNormalsEstimation", "Submodule DifferenceOfNormalsEstimation");
    defineFeaturesDifferenceOfNormalsEstimation<PointNormal, Normal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointNormal_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointNormal, Normal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointNormal_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<PointNormal, PointNormal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointNormal_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointNormal, PointNormal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointNormal_PointNormal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZ, Normal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZ_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZ, Normal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZ_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZ, PointNormal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZ_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZ_PointNormal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZI, Normal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZI_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZI, Normal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZI_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZI, PointNormal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZI_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZI, PointNormal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZI_PointNormal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZRGB, Normal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGB_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZRGB, Normal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGB_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGB_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGB_PointNormal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZRGBA, Normal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGBA_Normal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZRGBA, Normal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGBA_Normal_PointNormal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZRGBA, PointNormal, Normal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGBA_PointNormal_Normal");
    defineFeaturesDifferenceOfNormalsEstimation<PointXYZRGBA, PointNormal, PointNormal>(sub_module_DifferenceOfNormalsEstimation, "PointXYZRGBA_PointNormal_PointNormal");
}