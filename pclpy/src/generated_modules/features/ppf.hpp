
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;


#include <pcl/features/ppf.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesPPFEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::PPFEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineFeaturesPpfFunctions1(py::module &m) {
    m.def("computePPFPairFeature", py::overload_cast<const Eigen::Vector4f &, const Eigen::Vector4f &, const Eigen::Vector4f &, const Eigen::Vector4f &, float &, float &, float &, float &> (&pcl::computePPFPairFeature), "p1"_a, "n1"_a, "p2"_a, "n2"_a, "f1"_a, "f2"_a, "f3"_a, "f4"_a);
}

void defineFeaturesPpfFunctions(py::module &m) {
    defineFeaturesPpfFunctions1(m);
}

void defineFeaturesPpfClasses(py::module &sub_module) {
    py::module sub_module_PPFEstimation = sub_module.def_submodule("PPFEstimation", "Submodule PPFEstimation");
    defineFeaturesPPFEstimation<pcl::PointNormal, pcl::Normal, pcl::PPFSignature>(sub_module_PPFEstimation, "PointNormal_Normal_PPFSignature");
    defineFeaturesPPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature>(sub_module_PPFEstimation, "PointNormal_PointNormal_PPFSignature");
    defineFeaturesPPFEstimation<pcl::PointXYZ, pcl::Normal, pcl::PPFSignature>(sub_module_PPFEstimation, "PointXYZ_Normal_PPFSignature");
    defineFeaturesPPFEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PPFSignature>(sub_module_PPFEstimation, "PointXYZ_PointNormal_PPFSignature");
    defineFeaturesPPFEstimation<pcl::PointXYZI, pcl::Normal, pcl::PPFSignature>(sub_module_PPFEstimation, "PointXYZI_Normal_PPFSignature");
    defineFeaturesPPFEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::PPFSignature>(sub_module_PPFEstimation, "PointXYZI_PointNormal_PPFSignature");
    defineFeaturesPPFEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PPFSignature>(sub_module_PPFEstimation, "PointXYZRGBA_Normal_PPFSignature");
    defineFeaturesPPFEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::PPFSignature>(sub_module_PPFEstimation, "PointXYZRGBA_PointNormal_PPFSignature");
    defineFeaturesPpfFunctions(sub_module);
}