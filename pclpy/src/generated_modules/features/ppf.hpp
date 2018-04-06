
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/features/ppf.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesPPFEstimation(py::module &m, std::string const & suffix) {
    using Class = PPFEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, FeatureFromNormals<PointInT,PointNT,PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineFeaturesPpfClasses(py::module &sub_module) {
    py::module sub_module_PPFEstimation = sub_module.def_submodule("PPFEstimation", "Submodule PPFEstimation");
    defineFeaturesPPFEstimation<PointNormal, Normal, PPFSignature>(sub_module_PPFEstimation, "PointNormal_Normal_PPFSignature");
    defineFeaturesPPFEstimation<PointNormal, PointNormal, PPFSignature>(sub_module_PPFEstimation, "PointNormal_PointNormal_PPFSignature");
    defineFeaturesPPFEstimation<PointXYZ, Normal, PPFSignature>(sub_module_PPFEstimation, "PointXYZ_Normal_PPFSignature");
    defineFeaturesPPFEstimation<PointXYZ, PointNormal, PPFSignature>(sub_module_PPFEstimation, "PointXYZ_PointNormal_PPFSignature");
    defineFeaturesPPFEstimation<PointXYZI, Normal, PPFSignature>(sub_module_PPFEstimation, "PointXYZI_Normal_PPFSignature");
    defineFeaturesPPFEstimation<PointXYZI, PointNormal, PPFSignature>(sub_module_PPFEstimation, "PointXYZI_PointNormal_PPFSignature");
    defineFeaturesPPFEstimation<PointXYZRGBA, Normal, PPFSignature>(sub_module_PPFEstimation, "PointXYZRGBA_Normal_PPFSignature");
    defineFeaturesPPFEstimation<PointXYZRGBA, PointNormal, PPFSignature>(sub_module_PPFEstimation, "PointXYZRGBA_PointNormal_PPFSignature");
}