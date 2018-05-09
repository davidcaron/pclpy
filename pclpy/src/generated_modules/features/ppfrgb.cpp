
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);
#include "../../make_opaque_vectors.hpp"

#include <pcl/features/ppfrgb.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesPPFRGBEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::PPFRGBEstimation<PointInT, PointNT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesPPFRGBRegionEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::PPFRGBRegionEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineFeaturesPpfrgbFunctions(py::module &m) {
}

void defineFeaturesPpfrgbClasses(py::module &sub_module) {
    py::module sub_module_PPFRGBEstimation = sub_module.def_submodule("PPFRGBEstimation", "Submodule PPFRGBEstimation");
    defineFeaturesPPFRGBEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBA_Normal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBA_PointNormal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBA_PointXYZRGBNormal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<pcl::PointXYZRGBNormal, pcl::Normal, pcl::PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBNormal_Normal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<pcl::PointXYZRGBNormal, pcl::PointNormal, pcl::PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBNormal_PointNormal_PPFRGBSignature");
    defineFeaturesPPFRGBEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PPFRGBSignature>(sub_module_PPFRGBEstimation, "PointXYZRGBNormal_PointXYZRGBNormal_PPFRGBSignature");
}