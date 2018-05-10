
#include <pcl/surface/surfel_smoothing.h>



template <typename PointT, typename PointNT>
void defineSurfaceSurfelSmoothing(py::module &m, std::string const & suffix) {
    using Class = pcl::SurfelSmoothing<PointT, PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using NormalCloud = Class::NormalCloud;
    using NormalCloudPtr = Class::NormalCloudPtr;
    using CloudKdTree = Class::CloudKdTree;
    using CloudKdTreePtr = Class::CloudKdTreePtr;
    py::class_<Class, pcl::PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<float>(), "a_scale"_a=0.01);
    cls.def("initCompute", &Class::initCompute);
    cls.def("smoothCloudIteration", &Class::smoothCloudIteration, "output_positions"_a, "output_normals"_a);
    cls.def("computeSmoothedCloud", &Class::computeSmoothedCloud, "output_positions"_a, "output_normals"_a);
    cls.def("smoothPoint", &Class::smoothPoint, "point_index"_a, "output_point"_a, "output_normal"_a);
    cls.def("extractSalientFeaturesBetweenScales", &Class::extractSalientFeaturesBetweenScales, "cloud2"_a, "cloud2_normals"_a, "output_features"_a);
    cls.def("setInputNormals", &Class::setInputNormals, "a_normals"_a);
    cls.def("setSearchMethod", &Class::setSearchMethod, "a_tree"_a);
        
}

void defineSurfaceSurfelSmoothingFunctions(py::module &m) {
}

void defineSurfaceSurfelSmoothingClasses(py::module &sub_module) {
    py::module sub_module_SurfelSmoothing = sub_module.def_submodule("SurfelSmoothing", "Submodule SurfelSmoothing");
    defineSurfaceSurfelSmoothing<pcl::InterestPoint, pcl::Normal>(sub_module_SurfelSmoothing, "InterestPoint_Normal");
    defineSurfaceSurfelSmoothing<pcl::InterestPoint, pcl::PointNormal>(sub_module_SurfelSmoothing, "InterestPoint_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::InterestPoint, pcl::PointSurfel>(sub_module_SurfelSmoothing, "InterestPoint_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::InterestPoint, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "InterestPoint_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::InterestPoint, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "InterestPoint_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::InterestPoint, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "InterestPoint_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointDEM, pcl::Normal>(sub_module_SurfelSmoothing, "PointDEM_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointDEM, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointDEM_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointDEM, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointDEM_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointDEM, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointDEM_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointDEM, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointDEM_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointDEM, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointDEM_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointNormal, pcl::Normal>(sub_module_SurfelSmoothing, "PointNormal_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointNormal, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointNormal_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointNormal, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointNormal_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointNormal, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointNormal_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointNormal, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointNormal_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointNormal, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointNormal_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointSurfel, pcl::Normal>(sub_module_SurfelSmoothing, "PointSurfel_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointSurfel, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointSurfel_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointSurfel, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointSurfel_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointSurfel, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointSurfel_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointSurfel, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointSurfel_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointSurfel, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointSurfel_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithRange, pcl::Normal>(sub_module_SurfelSmoothing, "PointWithRange_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointWithRange, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointWithRange_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithRange, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointWithRange_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointWithRange, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointWithRange_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithRange, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointWithRange_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithRange, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointWithRange_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithScale, pcl::Normal>(sub_module_SurfelSmoothing, "PointWithScale_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointWithScale, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointWithScale_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithScale, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointWithScale_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointWithScale, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointWithScale_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithScale, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointWithScale_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithScale, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointWithScale_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithViewpoint, pcl::Normal>(sub_module_SurfelSmoothing, "PointWithViewpoint_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointWithViewpoint, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithViewpoint, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointWithViewpoint, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithViewpoint, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointWithViewpoint, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZ, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZ_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZ, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZ_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZ, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZ_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZ, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZ_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZ, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZ_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZ, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZ_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZHSV, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZHSV_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZHSV, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZHSV_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZHSV, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZHSV_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZHSV, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZHSV_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZHSV, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZHSV_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZHSV, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZHSV_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZI, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZI_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZI, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZI_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZI, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZI_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZI, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZI_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZI, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZI_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZI, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZI_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZINormal, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZINormal_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZINormal, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZINormal_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZINormal, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZINormal_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZINormal, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZINormal_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZINormal, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZINormal_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZINormal, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZINormal_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZL, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZL_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZL, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZL_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZL, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZL_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZL, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZL_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZL, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZL_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZL, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZL_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZLNormal, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZLNormal_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZLNormal, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZLNormal, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZLNormal, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZLNormal, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZLNormal, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGB, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZRGB_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGB, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZRGB_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGB, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZRGB_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGB, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZRGB_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGB, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZRGB_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZRGB_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBA, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZRGBA_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBA, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBA, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBA, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBA, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBA, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBL, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZRGBL_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBL, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBL, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBL, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBL, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBL, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBNormal, pcl::Normal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_Normal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBNormal, pcl::PointNormal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBNormal, pcl::PointSurfel>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointSurfel");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBNormal, pcl::PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointXYZINormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBNormal, pcl::PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointXYZRGBNormal");
}