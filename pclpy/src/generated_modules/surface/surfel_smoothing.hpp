
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/surface/surfel_smoothing.h>



template <typename PointT, typename PointNT>
void defineSurfaceSurfelSmoothing(py::module &m, std::string const & suffix) {
    using Class = SurfelSmoothing<PointT, PointNT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    using PointCloudIn = Class::PointCloudIn;
    using PointCloudInPtr = Class::PointCloudInPtr;
    using NormalCloud = Class::NormalCloud;
    using NormalCloudPtr = Class::NormalCloudPtr;
    using CloudKdTree = Class::CloudKdTree;
    using CloudKdTreePtr = Class::CloudKdTreePtr;
    py::class_<Class, PCLBase<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<float>(), "a_scale"_a=0.01);
    cls.def("set_input_normals", &Class::setInputNormals);
    cls.def("set_search_method", &Class::setSearchMethod);
    cls.def("init_compute", &Class::initCompute);
    cls.def("smooth_cloud_iteration", &Class::smoothCloudIteration);
    cls.def("compute_smoothed_cloud", &Class::computeSmoothedCloud);
    cls.def("smooth_point", &Class::smoothPoint);
    cls.def("extract_salient_features_between_scales", &Class::extractSalientFeaturesBetweenScales);
        
}

void defineSurfaceSurfelSmoothingClasses(py::module &sub_module) {
    py::module sub_module_SurfelSmoothing = sub_module.def_submodule("SurfelSmoothing", "Submodule SurfelSmoothing");
    defineSurfaceSurfelSmoothing<InterestPoint, Normal>(sub_module_SurfelSmoothing, "InterestPoint_Normal");
    defineSurfaceSurfelSmoothing<InterestPoint, PointNormal>(sub_module_SurfelSmoothing, "InterestPoint_PointNormal");
    defineSurfaceSurfelSmoothing<InterestPoint, PointSurfel>(sub_module_SurfelSmoothing, "InterestPoint_PointSurfel");
    defineSurfaceSurfelSmoothing<InterestPoint, PointXYZINormal>(sub_module_SurfelSmoothing, "InterestPoint_PointXYZINormal");
    defineSurfaceSurfelSmoothing<InterestPoint, PointXYZLNormal>(sub_module_SurfelSmoothing, "InterestPoint_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<InterestPoint, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "InterestPoint_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointDEM, Normal>(sub_module_SurfelSmoothing, "PointDEM_Normal");
    defineSurfaceSurfelSmoothing<PointDEM, PointNormal>(sub_module_SurfelSmoothing, "PointDEM_PointNormal");
    defineSurfaceSurfelSmoothing<PointDEM, PointSurfel>(sub_module_SurfelSmoothing, "PointDEM_PointSurfel");
    defineSurfaceSurfelSmoothing<PointDEM, PointXYZINormal>(sub_module_SurfelSmoothing, "PointDEM_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointDEM, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointDEM_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointDEM, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointDEM_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointNormal, Normal>(sub_module_SurfelSmoothing, "PointNormal_Normal");
    defineSurfaceSurfelSmoothing<PointNormal, PointNormal>(sub_module_SurfelSmoothing, "PointNormal_PointNormal");
    defineSurfaceSurfelSmoothing<PointNormal, PointSurfel>(sub_module_SurfelSmoothing, "PointNormal_PointSurfel");
    defineSurfaceSurfelSmoothing<PointNormal, PointXYZINormal>(sub_module_SurfelSmoothing, "PointNormal_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointNormal, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointNormal_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointNormal, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointNormal_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointSurfel, Normal>(sub_module_SurfelSmoothing, "PointSurfel_Normal");
    defineSurfaceSurfelSmoothing<PointSurfel, PointNormal>(sub_module_SurfelSmoothing, "PointSurfel_PointNormal");
    defineSurfaceSurfelSmoothing<PointSurfel, PointSurfel>(sub_module_SurfelSmoothing, "PointSurfel_PointSurfel");
    defineSurfaceSurfelSmoothing<PointSurfel, PointXYZINormal>(sub_module_SurfelSmoothing, "PointSurfel_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointSurfel, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointSurfel_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointSurfel, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointSurfel_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointWithRange, Normal>(sub_module_SurfelSmoothing, "PointWithRange_Normal");
    defineSurfaceSurfelSmoothing<PointWithRange, PointNormal>(sub_module_SurfelSmoothing, "PointWithRange_PointNormal");
    defineSurfaceSurfelSmoothing<PointWithRange, PointSurfel>(sub_module_SurfelSmoothing, "PointWithRange_PointSurfel");
    defineSurfaceSurfelSmoothing<PointWithRange, PointXYZINormal>(sub_module_SurfelSmoothing, "PointWithRange_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointWithRange, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointWithRange_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointWithRange, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointWithRange_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointWithScale, Normal>(sub_module_SurfelSmoothing, "PointWithScale_Normal");
    defineSurfaceSurfelSmoothing<PointWithScale, PointNormal>(sub_module_SurfelSmoothing, "PointWithScale_PointNormal");
    defineSurfaceSurfelSmoothing<PointWithScale, PointSurfel>(sub_module_SurfelSmoothing, "PointWithScale_PointSurfel");
    defineSurfaceSurfelSmoothing<PointWithScale, PointXYZINormal>(sub_module_SurfelSmoothing, "PointWithScale_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointWithScale, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointWithScale_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointWithScale, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointWithScale_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointWithViewpoint, Normal>(sub_module_SurfelSmoothing, "PointWithViewpoint_Normal");
    defineSurfaceSurfelSmoothing<PointWithViewpoint, PointNormal>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointNormal");
    defineSurfaceSurfelSmoothing<PointWithViewpoint, PointSurfel>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointSurfel");
    defineSurfaceSurfelSmoothing<PointWithViewpoint, PointXYZINormal>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointWithViewpoint, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointWithViewpoint, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointWithViewpoint_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZ, Normal>(sub_module_SurfelSmoothing, "PointXYZ_Normal");
    defineSurfaceSurfelSmoothing<PointXYZ, PointNormal>(sub_module_SurfelSmoothing, "PointXYZ_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZ, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZ_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZ, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZ_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZ, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZ_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZ, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZ_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZHSV, Normal>(sub_module_SurfelSmoothing, "PointXYZHSV_Normal");
    defineSurfaceSurfelSmoothing<PointXYZHSV, PointNormal>(sub_module_SurfelSmoothing, "PointXYZHSV_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZHSV, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZHSV_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZHSV, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZHSV_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZHSV, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZHSV_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZHSV, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZHSV_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZI, Normal>(sub_module_SurfelSmoothing, "PointXYZI_Normal");
    defineSurfaceSurfelSmoothing<PointXYZI, PointNormal>(sub_module_SurfelSmoothing, "PointXYZI_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZI, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZI_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZI, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZI_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZI, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZI_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZI, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZI_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZINormal, Normal>(sub_module_SurfelSmoothing, "PointXYZINormal_Normal");
    defineSurfaceSurfelSmoothing<PointXYZINormal, PointNormal>(sub_module_SurfelSmoothing, "PointXYZINormal_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZINormal, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZINormal_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZINormal, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZINormal_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZINormal, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZINormal_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZINormal, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZINormal_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZL, Normal>(sub_module_SurfelSmoothing, "PointXYZL_Normal");
    defineSurfaceSurfelSmoothing<PointXYZL, PointNormal>(sub_module_SurfelSmoothing, "PointXYZL_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZL, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZL_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZL, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZL_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZL, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZL_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZL, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZL_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZLNormal, Normal>(sub_module_SurfelSmoothing, "PointXYZLNormal_Normal");
    defineSurfaceSurfelSmoothing<PointXYZLNormal, PointNormal>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZLNormal, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZLNormal, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZLNormal, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZLNormal, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZLNormal_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGB, Normal>(sub_module_SurfelSmoothing, "PointXYZRGB_Normal");
    defineSurfaceSurfelSmoothing<PointXYZRGB, PointNormal>(sub_module_SurfelSmoothing, "PointXYZRGB_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGB, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZRGB_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZRGB, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZRGB_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZRGB, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZRGB_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGB, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZRGB_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBA, Normal>(sub_module_SurfelSmoothing, "PointXYZRGBA_Normal");
    defineSurfaceSurfelSmoothing<PointXYZRGBA, PointNormal>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBA, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZRGBA, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBA, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBA, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZRGBA_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBL, Normal>(sub_module_SurfelSmoothing, "PointXYZRGBL_Normal");
    defineSurfaceSurfelSmoothing<PointXYZRGBL, PointNormal>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBL, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZRGBL, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBL, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBL, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZRGBL_PointXYZRGBNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBNormal, Normal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_Normal");
    defineSurfaceSurfelSmoothing<PointXYZRGBNormal, PointNormal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBNormal, PointSurfel>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointSurfel");
    defineSurfaceSurfelSmoothing<PointXYZRGBNormal, PointXYZINormal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointXYZINormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBNormal, PointXYZLNormal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointXYZLNormal");
    defineSurfaceSurfelSmoothing<PointXYZRGBNormal, PointXYZRGBNormal>(sub_module_SurfelSmoothing, "PointXYZRGBNormal_PointXYZRGBNormal");
}