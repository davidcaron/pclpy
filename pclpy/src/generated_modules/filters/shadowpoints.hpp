
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace py = pybind11;
using namespace pybind11::literals;

using namespace pcl;


#include <pcl/filters/shadowpoints.h>



template<typename PointT, typename NormalT>
void defineFiltersShadowPoints(py::module &m, std::string const & suffix) {
    using Class = ShadowPoints<PointT, NormalT>;
    using Ptr = Class::Ptr;
    using ConstPtr = Class::ConstPtr;
    py::class_<Class, FilterIndices<PointT>, boost::shared_ptr<Class>> cls(m, suffix.c_str());
    cls.def(py::init<bool>(), "extract_removed_indices"_a=false);
    cls.def_property("normals", &Class::getNormals, &Class::setNormals);
    cls.def_property("threshold", &Class::getThreshold, &Class::setThreshold);
        
}

void defineFiltersShadowpointsClasses(py::module &sub_module) {
    py::module sub_module_ShadowPoints = sub_module.def_submodule("ShadowPoints", "Submodule ShadowPoints");
    defineFiltersShadowPoints<InterestPoint, Normal>(sub_module_ShadowPoints, "InterestPoint_Normal");
    defineFiltersShadowPoints<InterestPoint, PointNormal>(sub_module_ShadowPoints, "InterestPoint_PointNormal");
    defineFiltersShadowPoints<InterestPoint, PointSurfel>(sub_module_ShadowPoints, "InterestPoint_PointSurfel");
    defineFiltersShadowPoints<InterestPoint, PointXYZINormal>(sub_module_ShadowPoints, "InterestPoint_PointXYZINormal");
    defineFiltersShadowPoints<InterestPoint, PointXYZLNormal>(sub_module_ShadowPoints, "InterestPoint_PointXYZLNormal");
    defineFiltersShadowPoints<InterestPoint, PointXYZRGBNormal>(sub_module_ShadowPoints, "InterestPoint_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointDEM, Normal>(sub_module_ShadowPoints, "PointDEM_Normal");
    defineFiltersShadowPoints<PointDEM, PointNormal>(sub_module_ShadowPoints, "PointDEM_PointNormal");
    defineFiltersShadowPoints<PointDEM, PointSurfel>(sub_module_ShadowPoints, "PointDEM_PointSurfel");
    defineFiltersShadowPoints<PointDEM, PointXYZINormal>(sub_module_ShadowPoints, "PointDEM_PointXYZINormal");
    defineFiltersShadowPoints<PointDEM, PointXYZLNormal>(sub_module_ShadowPoints, "PointDEM_PointXYZLNormal");
    defineFiltersShadowPoints<PointDEM, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointDEM_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointNormal, Normal>(sub_module_ShadowPoints, "PointNormal_Normal");
    defineFiltersShadowPoints<PointNormal, PointNormal>(sub_module_ShadowPoints, "PointNormal_PointNormal");
    defineFiltersShadowPoints<PointNormal, PointSurfel>(sub_module_ShadowPoints, "PointNormal_PointSurfel");
    defineFiltersShadowPoints<PointNormal, PointXYZINormal>(sub_module_ShadowPoints, "PointNormal_PointXYZINormal");
    defineFiltersShadowPoints<PointNormal, PointXYZLNormal>(sub_module_ShadowPoints, "PointNormal_PointXYZLNormal");
    defineFiltersShadowPoints<PointNormal, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointNormal_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointSurfel, Normal>(sub_module_ShadowPoints, "PointSurfel_Normal");
    defineFiltersShadowPoints<PointSurfel, PointNormal>(sub_module_ShadowPoints, "PointSurfel_PointNormal");
    defineFiltersShadowPoints<PointSurfel, PointSurfel>(sub_module_ShadowPoints, "PointSurfel_PointSurfel");
    defineFiltersShadowPoints<PointSurfel, PointXYZINormal>(sub_module_ShadowPoints, "PointSurfel_PointXYZINormal");
    defineFiltersShadowPoints<PointSurfel, PointXYZLNormal>(sub_module_ShadowPoints, "PointSurfel_PointXYZLNormal");
    defineFiltersShadowPoints<PointSurfel, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointSurfel_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointWithRange, Normal>(sub_module_ShadowPoints, "PointWithRange_Normal");
    defineFiltersShadowPoints<PointWithRange, PointNormal>(sub_module_ShadowPoints, "PointWithRange_PointNormal");
    defineFiltersShadowPoints<PointWithRange, PointSurfel>(sub_module_ShadowPoints, "PointWithRange_PointSurfel");
    defineFiltersShadowPoints<PointWithRange, PointXYZINormal>(sub_module_ShadowPoints, "PointWithRange_PointXYZINormal");
    defineFiltersShadowPoints<PointWithRange, PointXYZLNormal>(sub_module_ShadowPoints, "PointWithRange_PointXYZLNormal");
    defineFiltersShadowPoints<PointWithRange, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointWithRange_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointWithScale, Normal>(sub_module_ShadowPoints, "PointWithScale_Normal");
    defineFiltersShadowPoints<PointWithScale, PointNormal>(sub_module_ShadowPoints, "PointWithScale_PointNormal");
    defineFiltersShadowPoints<PointWithScale, PointSurfel>(sub_module_ShadowPoints, "PointWithScale_PointSurfel");
    defineFiltersShadowPoints<PointWithScale, PointXYZINormal>(sub_module_ShadowPoints, "PointWithScale_PointXYZINormal");
    defineFiltersShadowPoints<PointWithScale, PointXYZLNormal>(sub_module_ShadowPoints, "PointWithScale_PointXYZLNormal");
    defineFiltersShadowPoints<PointWithScale, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointWithScale_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointWithViewpoint, Normal>(sub_module_ShadowPoints, "PointWithViewpoint_Normal");
    defineFiltersShadowPoints<PointWithViewpoint, PointNormal>(sub_module_ShadowPoints, "PointWithViewpoint_PointNormal");
    defineFiltersShadowPoints<PointWithViewpoint, PointSurfel>(sub_module_ShadowPoints, "PointWithViewpoint_PointSurfel");
    defineFiltersShadowPoints<PointWithViewpoint, PointXYZINormal>(sub_module_ShadowPoints, "PointWithViewpoint_PointXYZINormal");
    defineFiltersShadowPoints<PointWithViewpoint, PointXYZLNormal>(sub_module_ShadowPoints, "PointWithViewpoint_PointXYZLNormal");
    defineFiltersShadowPoints<PointWithViewpoint, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointWithViewpoint_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZ, Normal>(sub_module_ShadowPoints, "PointXYZ_Normal");
    defineFiltersShadowPoints<PointXYZ, PointNormal>(sub_module_ShadowPoints, "PointXYZ_PointNormal");
    defineFiltersShadowPoints<PointXYZ, PointSurfel>(sub_module_ShadowPoints, "PointXYZ_PointSurfel");
    defineFiltersShadowPoints<PointXYZ, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZ_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZ, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZ_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZ, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZ_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZHSV, Normal>(sub_module_ShadowPoints, "PointXYZHSV_Normal");
    defineFiltersShadowPoints<PointXYZHSV, PointNormal>(sub_module_ShadowPoints, "PointXYZHSV_PointNormal");
    defineFiltersShadowPoints<PointXYZHSV, PointSurfel>(sub_module_ShadowPoints, "PointXYZHSV_PointSurfel");
    defineFiltersShadowPoints<PointXYZHSV, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZHSV_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZHSV, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZHSV_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZHSV, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZHSV_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZI, Normal>(sub_module_ShadowPoints, "PointXYZI_Normal");
    defineFiltersShadowPoints<PointXYZI, PointNormal>(sub_module_ShadowPoints, "PointXYZI_PointNormal");
    defineFiltersShadowPoints<PointXYZI, PointSurfel>(sub_module_ShadowPoints, "PointXYZI_PointSurfel");
    defineFiltersShadowPoints<PointXYZI, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZI_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZI, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZI_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZI, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZI_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZINormal, Normal>(sub_module_ShadowPoints, "PointXYZINormal_Normal");
    defineFiltersShadowPoints<PointXYZINormal, PointNormal>(sub_module_ShadowPoints, "PointXYZINormal_PointNormal");
    defineFiltersShadowPoints<PointXYZINormal, PointSurfel>(sub_module_ShadowPoints, "PointXYZINormal_PointSurfel");
    defineFiltersShadowPoints<PointXYZINormal, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZINormal_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZINormal, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZINormal_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZINormal, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZINormal_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZL, Normal>(sub_module_ShadowPoints, "PointXYZL_Normal");
    defineFiltersShadowPoints<PointXYZL, PointNormal>(sub_module_ShadowPoints, "PointXYZL_PointNormal");
    defineFiltersShadowPoints<PointXYZL, PointSurfel>(sub_module_ShadowPoints, "PointXYZL_PointSurfel");
    defineFiltersShadowPoints<PointXYZL, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZL_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZL, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZL_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZL, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZL_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZLNormal, Normal>(sub_module_ShadowPoints, "PointXYZLNormal_Normal");
    defineFiltersShadowPoints<PointXYZLNormal, PointNormal>(sub_module_ShadowPoints, "PointXYZLNormal_PointNormal");
    defineFiltersShadowPoints<PointXYZLNormal, PointSurfel>(sub_module_ShadowPoints, "PointXYZLNormal_PointSurfel");
    defineFiltersShadowPoints<PointXYZLNormal, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZLNormal_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZLNormal, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZLNormal_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZLNormal, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZLNormal_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZRGB, Normal>(sub_module_ShadowPoints, "PointXYZRGB_Normal");
    defineFiltersShadowPoints<PointXYZRGB, PointNormal>(sub_module_ShadowPoints, "PointXYZRGB_PointNormal");
    defineFiltersShadowPoints<PointXYZRGB, PointSurfel>(sub_module_ShadowPoints, "PointXYZRGB_PointSurfel");
    defineFiltersShadowPoints<PointXYZRGB, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZRGB_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZRGB, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZRGB_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZRGB, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZRGB_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZRGBA, Normal>(sub_module_ShadowPoints, "PointXYZRGBA_Normal");
    defineFiltersShadowPoints<PointXYZRGBA, PointNormal>(sub_module_ShadowPoints, "PointXYZRGBA_PointNormal");
    defineFiltersShadowPoints<PointXYZRGBA, PointSurfel>(sub_module_ShadowPoints, "PointXYZRGBA_PointSurfel");
    defineFiltersShadowPoints<PointXYZRGBA, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZRGBA_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZRGBA, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZRGBA_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZRGBA, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZRGBA_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZRGBL, Normal>(sub_module_ShadowPoints, "PointXYZRGBL_Normal");
    defineFiltersShadowPoints<PointXYZRGBL, PointNormal>(sub_module_ShadowPoints, "PointXYZRGBL_PointNormal");
    defineFiltersShadowPoints<PointXYZRGBL, PointSurfel>(sub_module_ShadowPoints, "PointXYZRGBL_PointSurfel");
    defineFiltersShadowPoints<PointXYZRGBL, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZRGBL_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZRGBL, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZRGBL_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZRGBL, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZRGBL_PointXYZRGBNormal");
    defineFiltersShadowPoints<PointXYZRGBNormal, Normal>(sub_module_ShadowPoints, "PointXYZRGBNormal_Normal");
    defineFiltersShadowPoints<PointXYZRGBNormal, PointNormal>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointNormal");
    defineFiltersShadowPoints<PointXYZRGBNormal, PointSurfel>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointSurfel");
    defineFiltersShadowPoints<PointXYZRGBNormal, PointXYZINormal>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointXYZINormal");
    defineFiltersShadowPoints<PointXYZRGBNormal, PointXYZLNormal>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointXYZLNormal");
    defineFiltersShadowPoints<PointXYZRGBNormal, PointXYZRGBNormal>(sub_module_ShadowPoints, "PointXYZRGBNormal_PointXYZRGBNormal");
}