
#include <pcl/features/cppf.h>



template <typename PointInT, typename PointNT, typename PointOutT>
void defineFeaturesCPPFEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::CPPFEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
        
}

void defineFeaturesCppfFunctions1(py::module &m) {
    m.def("computeCPPFPairFeature", py::overload_cast<const Eigen::Vector4f &, const Eigen::Vector4f &, const Eigen::Vector4i &, const Eigen::Vector4f &, const Eigen::Vector4f &, const Eigen::Vector4i &, float &, float &, float &, float &, float &, float &, float &, float &, float &, float &> (&pcl::computeCPPFPairFeature), "p1"_a, "n1"_a, "c1"_a, "p2"_a, "n2"_a, "c2"_a, "f1"_a, "f2"_a, "f3"_a, "f4"_a, "f5"_a, "f6"_a, "f7"_a, "f8"_a, "f9"_a, "f10"_a);
}

void defineFeaturesCppfFunctions(py::module &m) {
    defineFeaturesCppfFunctions1(m);
}

void defineFeaturesCppfClasses(py::module &sub_module) {
    py::module sub_module_CPPFEstimation = sub_module.def_submodule("CPPFEstimation", "Submodule CPPFEstimation");
    defineFeaturesCPPFEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBA_Normal_CPPFSignature");
    defineFeaturesCPPFEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBA_PointNormal_CPPFSignature");
    defineFeaturesCPPFEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBA_PointXYZRGBNormal_CPPFSignature");
    defineFeaturesCPPFEstimation<pcl::PointXYZRGBNormal, pcl::Normal, pcl::CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBNormal_Normal_CPPFSignature");
    defineFeaturesCPPFEstimation<pcl::PointXYZRGBNormal, pcl::PointNormal, pcl::CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBNormal_PointNormal_CPPFSignature");
    defineFeaturesCPPFEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::CPPFSignature>(sub_module_CPPFEstimation, "PointXYZRGBNormal_PointXYZRGBNormal_CPPFSignature");
    defineFeaturesCppfFunctions(sub_module);
}