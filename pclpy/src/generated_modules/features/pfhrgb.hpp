
#include <pcl/features/pfhrgb.h>



template <typename PointInT, typename PointNT, typename PointOutT = pcl::PFHRGBSignature250>
void defineFeaturesPFHRGBEstimation(py::module &m, std::string const & suffix) {
    using Class = pcl::PFHRGBEstimation<PointInT, PointNT, PointOutT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudOut = Class::PointCloudOut;
    py::class_<Class, pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<>());
    cls.def("computeRGBPairFeatures", &Class::computeRGBPairFeatures, "cloud"_a, "normals"_a, "p_idx"_a, "q_idx"_a, "f1"_a, "f2"_a, "f3"_a, "f4"_a, "f5"_a, "f6"_a, "f7"_a);
    cls.def("computePointPFHRGBSignature", &Class::computePointPFHRGBSignature, "cloud"_a, "normals"_a, "indices"_a, "nr_split"_a, "pfhrgb_histogram"_a);
        
}

void defineFeaturesPfhrgbFunctions(py::module &m) {
}

void defineFeaturesPfhrgbClasses(py::module &sub_module) {
    py::module sub_module_PFHRGBEstimation = sub_module.def_submodule("PFHRGBEstimation", "Submodule PFHRGBEstimation");
    defineFeaturesPFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGB_Normal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGB_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGBA_Normal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal, pcl::PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGBA_PointXYZRGBNormal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<pcl::PointXYZRGBNormal, pcl::Normal, pcl::PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGBNormal_Normal_PFHRGBSignature250");
    defineFeaturesPFHRGBEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PFHRGBSignature250>(sub_module_PFHRGBEstimation, "PointXYZRGBNormal_PointXYZRGBNormal_PFHRGBSignature250");
}