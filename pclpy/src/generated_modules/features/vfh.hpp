
#include <pcl/features/vfh.h>



template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
void defineFeaturesVFHEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::VFHEstimation<PointInT, PointNT, PointOutT>;
    using PointCloudOut = Class::PointCloudOut;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computePointSPFHSignature", &Class::computePointSPFHSignature, "centroid_p"_a, "centroid_n"_a, "cloud"_a, "normals"_a, "indices"_a);
    cls.def("compute", &Class::compute, "output"_a);
    cls.def("setViewPoint", &Class::setViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
    cls.def("setUseGivenNormal", &Class::setUseGivenNormal, "use"_a);
    cls.def("setNormalToUse", &Class::setNormalToUse, "normal"_a);
    cls.def("setUseGivenCentroid", &Class::setUseGivenCentroid, "use"_a);
    cls.def("setCentroidToUse", &Class::setCentroidToUse, "centroid"_a);
    cls.def("setNormalizeBins", &Class::setNormalizeBins, "normalize"_a);
    cls.def("setNormalizeDistance", &Class::setNormalizeDistance, "normalize"_a);
    cls.def("setFillSizeComponent", &Class::setFillSizeComponent, "fill_size"_a);
    cls.def("getViewPoint", &Class::getViewPoint, "vpx"_a, "vpy"_a, "vpz"_a);
        
}

void defineFeaturesVfhFunctions(py::module &m) {
}

void defineFeaturesVfhClasses(py::module &sub_module) {
    py::module sub_module_VFHEstimation = sub_module.def_submodule("VFHEstimation", "Submodule VFHEstimation");
    defineFeaturesVFHEstimation<pcl::PointNormal, pcl::Normal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointNormal_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointNormal_PointNormal_VFHSignature308");
    defineFeaturesVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointXYZ_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointXYZ_PointNormal_VFHSignature308");
    defineFeaturesVFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointXYZI_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointXYZI_PointNormal_VFHSignature308");
    defineFeaturesVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointXYZRGB_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointXYZRGB_PointNormal_VFHSignature308");
    defineFeaturesVFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointXYZRGBA_Normal_VFHSignature308");
    defineFeaturesVFHEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::VFHSignature308>(sub_module_VFHEstimation, "PointXYZRGBA_PointNormal_VFHSignature308");
}