
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/ppfrgb.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesPPFRGBEstimation(py::module &m, std::string const & suffix) {
    using Class = PPFRGBEstimation<PointInT, PointNT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesPPFRGBRegionEstimation(py::module &m, std::string const & suffix) {
    using Class = PPFRGBRegionEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineFeaturesPpfrgbClasses(py::module &sub_module) {
    py::module sub_module_PPFRGBEstimation = sub_module.def_submodule("PPFRGBEstimation", "Submodule PPFRGBEstimation");
    defineFeaturesPPFRGBEstimation<PointXYZRGBA, Normal, PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBA_Normal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<PointXYZRGBA, PointNormal, PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBA_PointNormal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<PointXYZRGBA, PointXYZRGBNormal, PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBA_PointXYZRGBNormal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<PointXYZRGBNormal, Normal, PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBNormal_Normal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<PointXYZRGBNormal, PointNormal, PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBNormal_PointNormal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<PointXYZRGBNormal, PointXYZRGBNormal, PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBNormal_PointXYZRGBNormal_PPFRGBSignature");
}